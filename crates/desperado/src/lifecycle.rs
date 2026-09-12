//! Internal lifecycle primitives for live SDR bridge threads.

use std::sync::{
    Arc,
    atomic::{AtomicBool, Ordering},
};

/// Forward an item without allowing a full Tokio queue to make cancellation block forever.
///
/// The item is retained while the queue is full, so normal operation remains lossless. Once
/// cancellation is requested or the consumer has gone away, the bridge exits instead.
pub(crate) fn send_or_cancel<T>(
    tx: &tokio::sync::mpsc::Sender<T>,
    cancelled: &Arc<AtomicBool>,
    mut item: T,
) -> bool {
    loop {
        if cancelled.load(Ordering::Acquire) {
            return false;
        }
        match tx.try_send(item) {
            Ok(()) => return true,
            Err(tokio::sync::mpsc::error::TrySendError::Closed(_)) => return false,
            Err(tokio::sync::mpsc::error::TrySendError::Full(returned)) => {
                item = returned;
                std::thread::sleep(std::time::Duration::from_millis(1));
            }
        }
    }
}

/// Join a bridge thread away from the async executor.
pub(crate) async fn join_bridge(handle: Option<std::thread::JoinHandle<()>>) {
    if let Some(handle) = handle {
        let _ = tokio::task::spawn_blocking(move || handle.join()).await;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn cancellation_unblocks_a_full_bridge_queue() {
        let (tx, _rx) = tokio::sync::mpsc::channel(1);
        tx.try_send(1).unwrap();
        let cancelled = Arc::new(AtomicBool::new(true));

        assert!(!send_or_cancel(&tx, &cancelled, 2));
    }
}
