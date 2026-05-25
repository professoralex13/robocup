use teensy4_bsp::hal::{
    flexpwm::{
        ClockSelect, FULL_RELOAD_VALUE_REGISTER, LoadMode, Output, Prescaler, Pwm, Submodule,
    },
    iomuxc::{
        consts::Const,
        flexpwm::{A, B, Pin},
    },
};

const SERVO_FREQUENCY: u32 = 50;
const IPG_FREQUENCY: u32 = 150_000_000;
const PRESCALER: u32 = 128;
const PWM_FREQUENCY: u32 = IPG_FREQUENCY / PRESCALER;

const FULL_REV_US: u32 = 1050;
const FULL_FWD_US: u32 = 1950;

const FULL_REV_COUNTS: u32 = FULL_REV_US * PWM_FREQUENCY / 1_000_000;
const FULL_FWD_COUNTS: u32 = FULL_FWD_US * PWM_FREQUENCY / 1_000_000;

fn map_to_counts(value: f32) -> i16 {
    FULL_REV_COUNTS as i16 + (value * (FULL_FWD_COUNTS - FULL_REV_COUNTS) as f32) as i16
}

const CYCLE_COUNTS: u32 = PWM_FREQUENCY / SERVO_FREQUENCY;

pub struct ServoController<const N: u8, const M: u8, PA, PB> {
    submodule: Submodule<N, M>,
    output_a: Output<PA>,
    output_b: Output<PB>,
}

impl<const N: u8, const M: u8, PA, PB> ServoController<N, M, PA, PB>
where
    PA: Pin<Module = Const<N>, Output = A, Submodule = Const<M>>,
    PB: Pin<Module = Const<N>, Output = B, Submodule = Const<M>>,
{
    pub fn new(pwm: &mut Pwm<N>, mut submodule: Submodule<N, M>, pin_a: PA, pin_b: PB) -> Self {
        submodule.set_debug_enable(true);
        submodule.set_wait_enable(true);
        submodule.set_clock_select(ClockSelect::Ipg);
        submodule.set_prescaler(Prescaler::Prescaler128);
        submodule.set_load_mode(LoadMode::reload_full());
        submodule.set_load_frequency(1);
        submodule.set_initial_count(&pwm, 0);
        submodule.set_value(FULL_RELOAD_VALUE_REGISTER, CYCLE_COUNTS as i16);

        let output_a = Output::new_a(pin_a);
        let output_b = Output::new_b(pin_b);

        output_a.set_turn_on(&submodule, 0);
        output_b.set_turn_on(&submodule, 0);

        output_a.set_turn_off(&submodule, map_to_counts(0.0));
        output_a.set_turn_off(&submodule, map_to_counts(0.0));

        Self {
            submodule,
            output_a,
            output_b,
        }
    }

    pub fn set_a_value(&mut self, speed: f32) {
        self.output_a
            .set_turn_off(&self.submodule, map_to_counts(speed));
    }

    pub fn set_b_value(&mut self, speed: f32) {
        self.output_b
            .set_turn_off(&self.submodule, map_to_counts(speed));
    }
}
