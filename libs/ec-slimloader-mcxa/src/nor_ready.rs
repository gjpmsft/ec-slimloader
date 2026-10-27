//! Bounded status polling before interpreting the external NOR boot journal.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub(crate) enum ReadyError {
    Timeout,
    Command,
    Address,
}

fn port_address(preceding_sizes_kib: &[u32]) -> Result<u32, ReadyError> {
    preceding_sizes_kib.iter().try_fold(0u32, |address, size| {
        size.checked_mul(1024)
            .and_then(|bytes| address.checked_add(bytes))
            .ok_or(ReadyError::Address)
    })
}

fn wait_for_ready(
    mut read_status: impl FnMut() -> Result<u8, ReadyError>,
    expired: impl FnMut() -> bool,
) -> Result<(), ReadyError> {
    wait_until(|| Ok(read_status()? & 0x01 == 0), expired)
}

fn wait_until(
    mut ready: impl FnMut() -> Result<bool, ReadyError>,
    mut expired: impl FnMut() -> bool,
) -> Result<(), ReadyError> {
    loop {
        if expired() {
            return Err(ReadyError::Timeout);
        }
        if ready()? {
            return Ok(());
        }
        core::hint::spin_loop();
    }
}

#[cfg(all(target_os = "none", feature = "mcxa5xx"))]
pub(super) fn wait_for_nor_ready() -> Result<(), ReadyError> {
    #[cfg(not(feature = "mcxa5xxevk"))]
    use embassy_mcxa::pac;
    use embassy_time::{Duration, Instant};

    // Allow an interrupted sector/block erase to finish, without resetting the NOR.
    // Share one deadline between WIP polling and controller-command waits.
    let deadline = Instant::now() + Duration::from_secs(5);
    #[cfg(feature = "mcxa5xxevk")]
    let preceding_sizes = [];
    #[cfg(not(feature = "mcxa5xxevk"))]
    let preceding_sizes = [
        pac::FLEXSPI0.flshcr0(0).read().flshsz(),
        pac::FLEXSPI0.flshcr0(1).read().flshsz(),
    ];
    let address = port_address(&preceding_sizes)?;

    wait_for_ready(|| read_status(address, deadline), || Instant::now() >= deadline)
}

#[cfg(all(target_os = "none", feature = "mcxa5xx"))]
fn read_status(address: u32, deadline: embassy_time::Instant) -> Result<u8, ReadyError> {
    use embassy_mcxa::flexspi::lookup::Command;
    use embassy_mcxa::pac;
    use embassy_time::Instant;

    let regs = pac::FLEXSPI0;
    wait_until(
        || Ok(regs.sts0().read().seqidle() == pac::flexspi::Seqidle::Value1),
        || Instant::now() >= deadline,
    )?;

    let chip = if cfg!(feature = "mcxa5xxevk") { 0 } else { 2 };
    regs.flshcr2(chip).modify(|r| r.set_clrinstrptr(true));
    regs.intr().write(|r| {
        r.set_ipcmderr(true);
        r.set_ipcmdge(true);
        r.set_ipcmddone(true);
    });
    regs.iprxfcr()
        .modify(|r| r.set_clriprxf(pac::flexspi::Clriprxf::Value1));
    regs.ipcr0().write(|r| r.set_sfar(address));
    regs.ipcr1().write(|r| {
        r.set_idatsz(1);
        r.set_iseqid(Command::ReadStatus as u8);
        r.set_iseqnum(0);
        r.set_iparen(false);
    });
    regs.ipcmd().write(|r| r.set_trg(pac::flexspi::Trg::Value1));

    wait_until(
        || {
            let flags = regs.intr().read();
            if flags.ipcmderr() || flags.ipcmdge() {
                return Err(ReadyError::Command);
            }
            Ok(flags.ipcmddone() && regs.sts0().read().seqidle() == pac::flexspi::Seqidle::Value1)
        },
        || Instant::now() >= deadline,
    )?;

    if regs.sts1().read().ipcmderrcode() != pac::flexspi::Ipcmderrcode::Val0 {
        return Err(ReadyError::Command);
    }
    Ok((regs.rfdr(0).read().rxdata() & 0xff) as u8)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_address_selects_the_configured_flash_port() {
        assert_eq!(port_address(&[]), Ok(0));
        assert_eq!(port_address(&[0, 0]), Ok(0));
        assert_eq!(port_address(&[2048, 2048]), Ok(0x0040_0000));
    }

    #[test]
    fn invalid_mapping_does_not_wrap_to_another_flash() {
        assert_eq!(port_address(&[u32::MAX]), Err(ReadyError::Address));
    }

    #[test]
    fn ready_status_completes_after_one_probe() {
        let mut probes = 0;
        let result = wait_for_ready(
            || {
                probes += 1;
                Ok(0)
            },
            || false,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(probes, 1);
    }

    #[test]
    fn keeps_polling_busy_status_until_ready() {
        let mut probes = 0;
        let result = wait_for_ready(
            || {
                probes += 1;
                assert!(probes <= 3);
                Ok(if probes == 3 { 0x02 } else { 0x01 })
            },
            || false,
        );
        assert_eq!(result, Ok(()));
        assert_eq!(probes, 3);
    }

    #[test]
    fn deadline_stops_a_permanently_busy_device() {
        let mut probes = 0;
        let mut clock_samples = 0;
        let result = wait_for_ready(
            || {
                probes += 1;
                Ok(0xff)
            },
            || {
                clock_samples += 1;
                clock_samples == 3
            },
        );
        assert_eq!(result, Err(ReadyError::Timeout));
        assert_eq!(probes, 2);
    }

    #[test]
    fn status_command_errors_are_not_treated_as_ready() {
        assert_eq!(
            wait_for_ready(|| Err(ReadyError::Command), || false),
            Err(ReadyError::Command)
        );
    }

    #[test]
    fn expired_deadline_prevents_another_hardware_access() {
        assert_eq!(
            wait_for_ready(|| panic!("must not access flash after the deadline"), || true),
            Err(ReadyError::Timeout)
        );
    }
}
