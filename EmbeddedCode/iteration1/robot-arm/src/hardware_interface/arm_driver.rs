use rppal::
{
    spi::{self, SlaveSelect, Bus}, 
    Mode, 
};


struct ArmDriver {
    mac1: Spi,
    mac2: Spi,
    mac3: Spi,
}


impl ArmDriver {
    pub fn new() -> ArmDriver 
    {
        ArmDriver 
        { mac1: Spi::new(Bus::Spi0, SlaveSelect::Ss0, 8_000_000, Mode::Mode), 
            mac2: (),
            mac3: () 
        }
    }
}