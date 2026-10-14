//! A one-way simulated radio link, driven by sim time.
//!
//! Everything runs on one laptop, but robots and the edge still only talk
//! through these links, so latency, loss, outages and byte counts are real
//! properties of the experiment rather than of the host machine.

use rand::{Rng, SeedableRng};
use rand_chacha::ChaCha8Rng;
use std::collections::VecDeque;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkConfig {
    /// One-way latency in seconds.
    pub latency: f32,
    /// Uniform extra delay in [0, jitter) seconds.
    pub jitter: f32,
    /// Probability that a message is silently dropped.
    pub loss: f32,
}

impl Default for LinkConfig {
    fn default() -> Self {
        Self {
            latency: 0.040,
            jitter: 0.010,
            loss: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LinkStats {
    pub sent_msgs: u64,
    pub sent_bytes: u64,
    pub dropped_msgs: u64,
    pub delivered_msgs: u64,
}

pub struct Link<T> {
    pub config: LinkConfig,
    /// When false the far end is unreachable and everything is dropped.
    pub up: bool,
    pub stats: LinkStats,
    /// (deliver_at, sent_at, msg), sorted by delivery time.
    in_flight: VecDeque<(f32, f32, T)>,
    rng: ChaCha8Rng,
}

impl<T> Link<T> {
    pub fn new(config: LinkConfig, seed: u64) -> Self {
        Self {
            config,
            up: true,
            stats: LinkStats::default(),
            in_flight: VecDeque::new(),
            rng: ChaCha8Rng::seed_from_u64(seed),
        }
    }

    /// Queue `msg` at sim time `now`; returns false if it was lost. Bytes
    /// are counted either way: the sender still paid for the airtime.
    pub fn send(&mut self, now: f32, msg: T, bytes: usize) -> bool {
        self.stats.sent_msgs += 1;
        self.stats.sent_bytes += bytes as u64;
        if !self.up || self.rng.gen::<f32>() < self.config.loss {
            self.stats.dropped_msgs += 1;
            return false;
        }
        let jitter = if self.config.jitter > 0.0 {
            self.rng.gen_range(0.0..self.config.jitter)
        } else {
            0.0
        };
        let at = now + self.config.latency + jitter;
        // Keep the queue sorted by delivery time (jitter can reorder).
        let idx = self.in_flight.partition_point(|(t, _, _)| *t <= at);
        self.in_flight.insert(idx, (at, now, msg));
        true
    }

    /// Packets in the air: (sent_at, deliver_at, msg).
    pub fn in_flight(&self) -> impl Iterator<Item = (f32, f32, &T)> {
        self.in_flight.iter().map(|(at, sent, m)| (*sent, *at, m))
    }

    /// Everything due at or before `now`, in delivery order.
    pub fn recv(&mut self, now: f32) -> Vec<T> {
        let mut out = Vec::new();
        while self.in_flight.front().is_some_and(|(t, _, _)| *t <= now) {
            if let Some((_, _, m)) = self.in_flight.pop_front() {
                out.push(m);
            }
        }
        if !self.up {
            // An outage also kills what was already in the air.
            self.stats.dropped_msgs += out.len() as u64;
            out.clear();
        }
        self.stats.delivered_msgs += out.len() as u64;
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn link(latency: f32, loss: f32) -> Link<u32> {
        Link::new(
            LinkConfig {
                latency,
                jitter: 0.0,
                loss,
            },
            1,
        )
    }

    #[test]
    fn delivers_after_latency() {
        let mut l = link(0.1, 0.0);
        l.send(0.0, 1, 10);
        assert!(l.recv(0.05).is_empty());
        assert_eq!(l.recv(0.1), vec![1]);
        assert_eq!(l.stats.sent_bytes, 10);
    }

    #[test]
    fn outage_drops_everything() {
        let mut l = link(0.1, 0.0);
        l.send(0.0, 1, 4);
        l.up = false;
        l.send(0.0, 2, 4);
        assert!(l.recv(1.0).is_empty());
        assert_eq!(l.stats.dropped_msgs, 2);
    }

    #[test]
    fn loss_is_roughly_honoured() {
        let mut l = link(0.0, 0.3);
        for i in 0..10_000 {
            l.send(0.0, i, 1);
        }
        let got = l.recv(0.0).len();
        assert!((6_500..7_500).contains(&got), "{got}");
    }
}
