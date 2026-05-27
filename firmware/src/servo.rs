use teensy4_bsp::hal::{
    flexpwm::{
        ClockSelect, FULL_RELOAD_VALUE_REGISTER, LoadMode, Output, PairOperation, Prescaler, Pwm,
        Submodule,
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
    FULL_REV_COUNTS as i16
        + (0.5 * (value + 1.0) * (FULL_FWD_COUNTS - FULL_REV_COUNTS) as f32) as i16
}

const CYCLE_COUNTS: u32 = PWM_FREQUENCY / SERVO_FREQUENCY;

pub struct ServoController<const N: u8, const M: u8, PA, PB> {
    submodule: Submodule<N, M>,
    output_a: Output<PA>,
    output_b: Output<PB>,

    pwm: Pwm<N>,
}

impl<const N: u8, const M: u8, PA, PB> ServoController<N, M, PA, PB>
where
    PA: Pin<Module = Const<N>, Output = A, Submodule = Const<M>>,
    PB: Pin<Module = Const<N>, Output = B, Submodule = Const<M>>,
{
    pub fn new(mut pwm: Pwm<N>, mut submodule: Submodule<N, M>, pin_a: PA, pin_b: PB) -> Self {
        submodule.set_debug_enable(true);
        submodule.set_wait_enable(true);
        submodule.set_clock_select(ClockSelect::Ipg);
        submodule.set_prescaler(Prescaler::Prescaler128);
        submodule.set_pair_operation(PairOperation::Independent);
        submodule.set_load_mode(LoadMode::reload_full());
        submodule.set_load_frequency(1);
        submodule.set_initial_count(&pwm, 0);
        submodule.set_value(FULL_RELOAD_VALUE_REGISTER, CYCLE_COUNTS as i16);

        let output_a = Output::new_a(pin_a);
        let output_b = Output::new_b(pin_b);

        output_a.set_turn_on(&submodule, 0);
        output_b.set_turn_on(&submodule, 0);

        output_a.set_turn_off(&submodule, map_to_counts(0.0));
        output_b.set_turn_off(&submodule, map_to_counts(0.0));

        output_a.set_output_enable(&mut pwm, true);
        output_b.set_output_enable(&mut pwm, true);

        submodule.set_load_ok(&mut pwm);
        submodule.set_running(&mut pwm, true);

        Self {
            submodule,
            output_a,
            output_b,
            pwm,
        }
    }

    pub fn set_values(&mut self, a_val: f32, b_val: f32) {
        self.output_a
            .set_turn_off(&self.submodule, map_to_counts(a_val));
        self.output_b
            .set_turn_off(&self.submodule, map_to_counts(b_val));
        self.submodule.set_load_ok(&mut self.pwm);
    }
}
