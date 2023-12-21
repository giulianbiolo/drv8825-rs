#![allow(dead_code)]
use rppal::gpio::{Gpio, Error, OutputPin};
use spin_sleep::SpinSleeper;
use trajectory::{CubicSpline, Trajectory};

const ONE_MILLIS: std::time::Duration = std::time::Duration::from_millis(1);

pub enum Direction { Forward, Reverse }
pub struct DRV8825 {
    enable: Option<OutputPin>,
    step: OutputPin,
    dir: OutputPin,
    microsteps: u8,
    radius: f64,
    accel: f64,
    cur_speed: f64,
}

impl DRV8825 {
    pub fn new(gpio: &Gpio, enable_pin: Option<u8>, step_pin: u8, dir_pin: u8, microsteps: u8, radius: f64, accel: f64) -> Result<DRV8825, Error> {
        if let Some(enable_pin) = enable_pin {
            let enable: OutputPin = gpio.get(enable_pin)?.into_output();
            let step: OutputPin = gpio.get(step_pin)?.into_output();
            let dir: OutputPin = gpio.get(dir_pin)?.into_output();
            Ok(DRV8825 { enable: Some(enable), step, dir, microsteps, radius, accel: accel/(microsteps as f64), cur_speed: 0.0 })
        } else {
            let step: OutputPin = gpio.get(step_pin)?.into_output();
            let dir: OutputPin = gpio.get(dir_pin)?.into_output();
            Ok(DRV8825 { enable: None, step, dir, microsteps, radius, accel: accel/(microsteps as f64), cur_speed: 0.0 })
        }
    }
    pub fn enable(&mut self) -> Result<(), Error> {
        if let Some(enable) = &mut self.enable { enable.set_low(); }
        Ok(())
    }
    pub fn disable(&mut self) -> Result<(), Error> {
        if let Some(enable) = &mut self.enable { enable.set_high(); }
        Ok(())
    }
    pub fn set_direction(&mut self, direction: Direction) -> Result<(), Error> {
        match direction {
            Direction::Forward => self.dir.set_high(),
            Direction::Reverse => self.dir.set_low(),
        }
        Ok(())
    }
    pub fn step(&mut self, spin_sleep: &SpinSleeper, timing_ns: u64) {
        self.step.set_high();
        spin_sleep.sleep_ns(timing_ns / 2);
        self.step.set_low();
        spin_sleep.sleep_ns(timing_ns / 2);
    }
    pub fn step_n(&mut self, spin_sleep: &SpinSleeper, n: u32) {
        for _ in 0..n { self.step(spin_sleep, ONE_MILLIS.as_nanos() as u64); }
    }
    pub fn step_n_timed(&mut self, spin_sleep: &SpinSleeper, n: u32, total_time: f64) {
        let timing_ns: u64 = ((total_time / (n as f64)) * 1_000_000_000_f64) as u64;
        for _ in 0..n { self.step(spin_sleep, timing_ns); }
    }
    // * steps_from_distance(distance) returns the number of steps required to move the given distance.
    fn steps_from_distance(&self, distance: f64) -> u32 {
        let circumference: f64 = 2.0 * std::f64::consts::PI * self.radius;
        let steps: f64 = (distance / circumference) * (self.microsteps as f64) * 200.0;
        steps as u32
    }
    fn distance_from_steps(&self, steps: u32) -> f64 {
        let circumference: f64 = 2.0 * std::f64::consts::PI * self.radius;
        let distance: f64 = (steps as f64) / (self.microsteps as f64) / 200.0 * circumference;
        distance
    }
    // * get_timing() returns the number of nanoseconds to wait between steps for the current speed.
    fn get_timing(&self) -> u64 { (1_000_000_000 / self.steps_from_distance(self.cur_speed).clamp(10, u32::MAX)) as u64 }

    // * travel(spin_sleep, distance, target_speed) moves the motor the given distance through the given target_speed.
    pub fn travel_ease_in(&mut self, spin_sleep: &SpinSleeper, distance: f64, travel_speed: f64) {
        // we want to spool up or down the motors from cur_speed to target_speed, reaching target_speed at min time (max accel) and at the end of the travel keep target_speed
        let accel_decel: f64 = (travel_speed - self.cur_speed).signum() as f64; // 1 for accel, -1 for decel
        println!("Steps: {}", self.steps_from_distance(distance));
        for idx in 0..self.steps_from_distance(distance) {
            if (accel_decel > 0.0 && self.cur_speed < travel_speed) || (accel_decel < 0.0 && self.cur_speed > travel_speed) {
                self.cur_speed += self.accel * accel_decel;
            }
            self.step(spin_sleep, self.get_timing());
            println!("IDX: {} | Cur Speed: {} | Timing: {} | Distance Travelled: {}", idx, self.cur_speed, self.get_timing(), self.distance_from_steps(idx));
        }
    }
    pub fn travel_ease_in_out(&mut self, spin_sleep: &SpinSleeper, distance: f64, travel_speed: f64, final_speed: f64) {
        // we want to spool up or down the motors from cur_speed to target_speed, reaching target_speed at min time (max accel) and at the end must come to a stop at min time (-max accel)
        let accel_decel: f64 = (travel_speed - self.cur_speed).signum() as f64; // 1 for accel, -1 for decel
        let steps: u32 = self.steps_from_distance(distance);
        //println!("Steps: {}", self.steps_from_distance(distance));
        let mut reached_idx: u32 = steps;
        for idx in 0..steps {
            // compute if we need to start decelerating
            if (accel_decel > 0.0 && self.cur_speed < travel_speed) || (accel_decel < 0.0 && self.cur_speed > travel_speed) {
                self.cur_speed += self.accel * accel_decel;
            }
            self.step(spin_sleep, self.get_timing());
            //println!("1) IDX: {} | Cur Speed: {} | Timing: {} | Distance Travelled: {}", idx, self.cur_speed, self.get_timing(), self.distance_from_steps(idx));
            let steps_to_final_speed: u32 = ((self.cur_speed - final_speed).abs() / self.accel) as u32;
            if steps_to_final_speed >= steps - idx {
                reached_idx = idx;
                break;
            }
            // delta speed / acceleration == step to final speed
            // if step to stop < step remaining in for loop (self.steps_from_distance(distance) - i) then we need to start decelerating
        }
        //println!("Remaining Steps: {}", steps - reached_idx);
        let accel_decel: f64 = (final_speed - self.cur_speed).signum() as f64; // 1 for accel, -1 for decel
        for idx in reached_idx..steps {
            if (accel_decel > 0.0 && self.cur_speed < final_speed) || (accel_decel < 0.0 && self.cur_speed > final_speed) {
                self.cur_speed += self.accel * accel_decel;
            }
            self.step(spin_sleep, self.get_timing());
            //println!("2) IDX: {} | Cur Speed: {} | Timing: {} | Distance Travelled: {}", idx, self.cur_speed, self.get_timing(), self.distance_from_steps(idx));
        }
        self.cur_speed = final_speed;
    }

    pub fn travel(&mut self, spin_sleep: &SpinSleeper, distance: Vec<f64>, time: Vec<f64>) {
        let dists = distance.iter().map(|d| vec![*d, 0.0_f64]).collect::<Vec<Vec<f64>>>();
        let tot_time = time[time.len() - 1];
        let mut times = vec![0.0_f64, 0.01_f64];
        times.extend(time);
        println!("Times: {:?}", times);
        let mut distances = vec![vec![0.0_f64, 0.0_f64], vec![self.cur_speed * 0.01_f64, 0.0_f64]];
        distances.extend(dists);
        let spline = CubicSpline::new(times, distances).unwrap();
        // sample time * 100 times, each sample is 0.01 seconds long
        let mut positions = Vec::new();
        for t in 1..(tot_time * 100.0_f64) as usize {
            let t = t as f64 * 0.01_f64;
            let p = spline.position(t).unwrap();
            // let v = spline.velocity(t).unwrap();
            // let a = spline.acceleration(t).unwrap();
            positions.push(p[0]);
        }
        // now we move the motor, sample 0.01 seconds at a time and move the motor of the distance between p and p-1
        for idx in 1..positions.len() {
            let distance = positions[idx] - positions[idx - 1];
            let steps = self.steps_from_distance(distance);
            self.step_n_timed(spin_sleep, steps, 0.01_f64);
        }
    }
}
