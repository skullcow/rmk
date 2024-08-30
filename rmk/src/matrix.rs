use crate::debounce::{DebounceState, DebouncerTrait};
use defmt::Format;
use embassy_time::{Instant, Timer};
use embedded_hal::digital::{InputPin, OutputPin, PinState};
#[cfg(feature = "async_matrix")]
use {
    defmt::info, embassy_futures::select::select_slice, embedded_hal_async::digital::Wait,
    heapless::Vec,
};

/// MatrixTrait is the trait for keyboard matrix.
///
/// The keyboard matrix is a 2D matrix of keys, the matrix does the scanning and saves the result to each key's `KeyState`.
/// The `KeyState` at position (row, col) can be read by `get_key_state` and updated by `update_key_state`.
pub(crate) trait MatrixTrait {
    // Do matrix scanning, save the result in matrix's key_state field.
    async fn scan(&mut self);
    // Read key state at position (row, col)
    fn get_key_state(&mut self, row: usize, col: usize) -> KeyState;
    // Update key state at position (row, col)
    fn update_key_state(&mut self, row: usize, col: usize, f: impl FnOnce(&mut KeyState));
    #[cfg(feature = "async_matrix")]
    async fn wait_for_key(&mut self);
}

/// KeyState represents the state of a key.
#[derive(Copy, Clone, Debug, Format)]
pub(crate) struct KeyState {
    // True if the key is pressed
    pub(crate) pressed: bool,
    // True if the key's state is just changed
    pub(crate) changed: bool,
    // If the key is held, `hold_start` records the time of it was pressed.
    pub(crate) hold_start: Option<Instant>,
}

impl Default for KeyState {
    fn default() -> Self {
        Self::new()
    }
}

impl KeyState {
    fn new() -> Self {
        KeyState {
            pressed: false,
            changed: false,
            hold_start: None,
        }
    }

    // Record the start time of pressing
    pub(crate) fn start_timer(&mut self) {
        self.hold_start = Some(Instant::now());
    }

    // Clear held timer
    pub(crate) fn clear_timer(&mut self) {
        self.hold_start = None;
    }

    pub(crate) fn toggle_pressed(&mut self) {
        self.pressed = !self.pressed;
    }

    pub(crate) fn is_releasing(&self) -> bool {
        !self.pressed && self.changed
    }

    pub(crate) fn is_pressing(&self) -> bool {
        self.pressed && self.changed
    }
}

/// Matrix is the physical pcb layout of the keyboard matrix.
pub struct Matrix<
    #[cfg(feature = "async_matrix")] In: Wait + InputPin,
    #[cfg(not(feature = "async_matrix"))] In: InputPin,
    Out: OutputPin,
    D: DebouncerTrait,
    const INPUT_PIN_NUM: usize,
    const OUTPUT_PIN_NUM: usize,
> {
    /// Input pins of the pcb matrix
    input_pins: [In; INPUT_PIN_NUM],
    /// Output pins of the pcb matrix
    output_pins: [Out; OUTPUT_PIN_NUM],
    /// Debouncer
    debouncer: D,
    /// Key state matrix
    key_states: [[KeyState; INPUT_PIN_NUM]; OUTPUT_PIN_NUM],
    /// Start scanning
    scan_start: Option<Instant>,
}

impl<
        #[cfg(not(feature = "async_matrix"))] In: InputPin,
        #[cfg(feature = "async_matrix")] In: Wait + InputPin,
        Out: OutputPin,
        D: DebouncerTrait,
        const INPUT_PIN_NUM: usize,
        const OUTPUT_PIN_NUM: usize,
    > Matrix<In, Out, D, INPUT_PIN_NUM, OUTPUT_PIN_NUM>
{
    /// Create a matrix from input and output pins.
    pub(crate) fn new(input_pins: [In; INPUT_PIN_NUM], output_pins: [Out; OUTPUT_PIN_NUM]) -> Self {
        Matrix {
            input_pins,
            output_pins,
            debouncer: D::new(),
            key_states: [[KeyState::new(); INPUT_PIN_NUM]; OUTPUT_PIN_NUM],
            scan_start: None,
        }
    }
}

impl<
        #[cfg(not(feature = "async_matrix"))] In: InputPin,
        #[cfg(feature = "async_matrix")] In: Wait + InputPin,
        Out: OutputPin,
        D: DebouncerTrait,
        const INPUT_PIN_NUM: usize,
        const OUTPUT_PIN_NUM: usize,
    > MatrixTrait for Matrix<In, Out, D, INPUT_PIN_NUM, OUTPUT_PIN_NUM>
{
    #[cfg(feature = "async_matrix")]
    async fn wait_for_key(&mut self) {
        if let Some(start_time) = self.scan_start {
            // If not key over 2 secs, wait for interupt in next loop
            if start_time.elapsed().as_secs() < 1 {
                return;
            } else {
                self.scan_start = None;
            }
        }
        // First, set all output pin to high
        for out in self.output_pins.iter_mut() {
            out.set_high().ok();
        }
        Timer::after_micros(1).await;
        info!("Waiting for high");
        let mut futs: Vec<_, INPUT_PIN_NUM> = self
            .input_pins
            .iter_mut()
            .map(|input_pin| input_pin.wait_for_high())
            .collect();
        let _ = select_slice(futs.as_mut_slice()).await;

        // Set all output pins back to low
        for out in self.output_pins.iter_mut() {
            out.set_low().ok();
        }

        self.scan_start = Some(Instant::now());
    }

    /// Do matrix scanning, the result is stored in matrix's key_state field.
    async fn scan(&mut self) {
        for (out_idx, out_pin) in self.output_pins.iter_mut().enumerate() {
            // Pull up output pin, wait 1us ensuring the change comes into effect
            out_pin.set_high().ok();
            Timer::after_micros(1).await;
            for (in_idx, in_pin) in self.input_pins.iter_mut().enumerate() {
                // Check input pins and debounce
                let debounce_state = self.debouncer.detect_change_with_debounce(
                    in_idx,
                    out_idx,
                    in_pin.is_high().ok().unwrap_or_default(),
                    &self.key_states[out_idx][in_idx],
                );

                match debounce_state {
                    DebounceState::Debounced => {
                        self.key_states[out_idx][in_idx].toggle_pressed();
                        self.key_states[out_idx][in_idx].changed = true;
                    }
                    _ => self.key_states[out_idx][in_idx].changed = false,
                }

                // If there's key changed or pressed, always refresh the self.scan_start
                #[cfg(feature = "async_matrix")]
                if self.key_states[out_idx][in_idx].changed
                    || self.key_states[out_idx][in_idx].pressed
                {
                    self.scan_start = Some(Instant::now());
                }
            }
            out_pin.set_low().ok();
        }
    }

    /// Read key state at position (row, col)
    fn get_key_state(&mut self, row: usize, col: usize) -> KeyState {
        // COL2ROW
        #[cfg(feature = "col2row")]
        return self.key_states[col][row];

        // ROW2COL
        #[cfg(not(feature = "col2row"))]
        return self.key_states[row][col];
    }

    fn update_key_state(&mut self, row: usize, col: usize, f: impl FnOnce(&mut KeyState)) {
        // COL2ROW
        #[cfg(feature = "col2row")]
        f(&mut self.key_states[col][row]);

        // ROW2COL
        #[cfg(not(feature = "col2row"))]
        f(&mut self.key_states[row][col]);
    }
}

pub(crate) enum BoardSide {
    Left,
    Right,
}

/// MuxMatrix
pub(crate) struct MuxMatrix<
    In: InputPin,
    Idx: OutputPin,
    Side: OutputPin,
    D: DebouncerTrait,
    const IDX_PIN_NUM: usize,
    const INPUT_PIN_NUM: usize,
    const OUTPUTS_PER_SIDE: usize,
    const TOTAL_OUTPUTS: usize, // **must be OUTPUTS_PER_SIDE * 2**
> {
    /// Input pins of the pcb matrix
    input_pins: [In; INPUT_PIN_NUM],
    /// Output index pins - will be used to set Multiplexer address
    idx_pins: [Idx; IDX_PIN_NUM],
    ///Side select pin, used to enable/disable multiplexer on each side.
    side_pin: Side,
    /// Debouncer
    debouncer: D,
    /// Key state matrix
    key_states: [[KeyState; INPUT_PIN_NUM]; TOTAL_OUTPUTS],
    /// Start scanning
    scan_start: Option<Instant>,
}

impl<
        // #[cfg(feature = "async_matrix")] In: Wait + InputPin, // Not sure if we need to support this yet
        In: InputPin,
        Idx: OutputPin,
        Side: OutputPin,
        D: DebouncerTrait,
        const IDX_PIN_NUM: usize,
        const INPUT_PIN_NUM: usize,
        const OUTPUTS_PER_SIDE: usize,
        const TOTAL_OUTPUTS: usize,
    > MuxMatrix<In, Idx, Side, D, IDX_PIN_NUM, INPUT_PIN_NUM, OUTPUTS_PER_SIDE, TOTAL_OUTPUTS>
{
    /// Create a matrix from input and output pins.
    pub(crate) fn new(
        input_pins: [In; INPUT_PIN_NUM],
        idx_pins: [Idx; IDX_PIN_NUM],
        side_pin: Side,
    ) -> Self {
        MuxMatrix {
            input_pins,
            idx_pins,
            side_pin,
            debouncer: D::new(),
            key_states: [[KeyState::new(); INPUT_PIN_NUM]; TOTAL_OUTPUTS],
            scan_start: None,
        }
    }

    pub fn set_side(&mut self, side: BoardSide) {
        match side {
            BoardSide::Left => self.side_pin.set_low().unwrap(),
            BoardSide::Right => self.side_pin.set_high().unwrap(),
        }
    }

    pub fn set_index(&mut self, idx: usize) {
        assert!(idx < OUTPUTS_PER_SIDE);
        for i in 0usize..IDX_PIN_NUM {
            let idx_bit = (idx & (1 >> i)) != 0;
            self.idx_pins[i].set_state(PinState::from(idx_bit)).unwrap();
        }
    }

    pub async fn set_output(&mut self, output: usize) {
        let (side, idx) = if output < OUTPUTS_PER_SIDE {
            (BoardSide::Left, output)
        } else {
            (BoardSide::Right, output - OUTPUTS_PER_SIDE)
        };
        self.set_side(side);
        self.set_index(idx);
        Timer::after_micros(1).await; // TODO: check timing requirements
    }
}

impl<
        // #[cfg(feature = "async_matrix")] In: Wait + InputPin, // Not sure if we need to support this yet
        In: InputPin,
        Idx: OutputPin,
        Side: OutputPin,
        D: DebouncerTrait,
        const IDX_PIN_NUM: usize,
        const INPUT_PIN_NUM: usize,
        const OUTPUTS_PER_SIDE: usize,
        const TOTAL_OUTPUTS: usize,
    > MatrixTrait
    for MuxMatrix<In, Idx, Side, D, IDX_PIN_NUM, INPUT_PIN_NUM, OUTPUTS_PER_SIDE, TOTAL_OUTPUTS>
{
    async fn scan(&mut self) {
        for out_idx in 0..(2 * OUTPUTS_PER_SIDE) {
            self.set_output(out_idx).await;
            for (in_idx, in_pin) in self.input_pins.iter_mut().enumerate() {
                // Check input pins and debounce
                let debounce_state = self.debouncer.detect_change_with_debounce(
                    in_idx,
                    out_idx,
                    in_pin.is_high().ok().unwrap_or_default(),
                    &self.key_states[out_idx][in_idx],
                );

                match debounce_state {
                    DebounceState::Debounced => {
                        self.key_states[out_idx][in_idx].toggle_pressed();
                        self.key_states[out_idx][in_idx].changed = true;
                    }
                    _ => self.key_states[out_idx][in_idx].changed = false,
                }
            }
        }
    }

    fn get_key_state(&mut self, row: usize, col: usize) -> KeyState {
        self.key_states[col][row]
    }

    fn update_key_state(&mut self, row: usize, col: usize, f: impl FnOnce(&mut KeyState)) {
        f(&mut self.key_states[col][row]);
    }

    #[cfg(feature = "async_matrix")]
    async fn wait_for_key(&mut self) {
        todo!()
    }
}
