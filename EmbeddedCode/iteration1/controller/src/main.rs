/*
William Albertini

This code is for the human interface to the 
robotic arm. It tries to establish a socket 
connection with a pre-defined IPV4 address 
and port. Once connection is established, 
joysticks are sampled at a given rate. When 
an input is detected, this data is sent over
the network to the main computer.
*/



// gpio imports
use esp_idf_svc::hal;
use hal::delay::Delay;
use hal::peripherals::Peripherals;
use hal::gpio::{
    PinDriver,
    Pull,
};
use hal::adc::{
    config::Config as AdcConfig,
    AdcDriver,
    AdcChannelDriver,
};

// wifi imports
use hal::modem::Modem;
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::wifi::{
    AuthMethod,
    Configuration,
    ClientConfiguration,
    EspWifi,
};
use std::net::{
    UdpSocket,
    SocketAddrV4,
};
use std::io::Write;

// internal imports
mod joystick_interface;
mod data_handler;
use joystick_interface::JoyStick;
use data_handler::DataHandler;

// wifi credentials
const SSID: &str = "will_ipad";
const PASSWORD: &str = "ubuntu707";
const IPADDR: &str = "172.20.10.3:8001";


fn main() -> anyhow::Result<()> {
    // necessary to call this function once
    esp_idf_svc::sys::link_patches();

    // make peripherals accessible
    let dp = Peripherals::take()?;
    // set up delays
    let delay = Delay::new(2000);

    // set up adc channels for joystick pins
    let mut adc1 = AdcDriver::new(dp.adc1, &AdcConfig::new().calibration(true))?;

    // ------------------------------  GPIO Pin Setup -------------------------------------------
    // set up joystick 1 (tranlational movement)
    let mut button1 = PinDriver::input(dp.pins.gpio10)?;
    button1.set_pull(Pull::Up)?;
    let mut joystick1 = JoyStick::new(
        AdcChannelDriver::new(dp.pins.gpio1)?,
        AdcChannelDriver::new(dp.pins.gpio0)?,
        button1,
    );

    // set ip joystick 2 (roll, pitch)
    let mut button2 = PinDriver::input(dp.pins.gpio9)?;
    button2.set_pull(Pull::Up)?;
    let mut joystick2 = JoyStick::new(
        AdcChannelDriver::new(dp.pins.gpio2)?,
        AdcChannelDriver::new(dp.pins.gpio3)?,
        button2,
    );

    // ------------------------------ WIFI Pin Setup -------------------------------------------
    // setup network connection
    let wifi = match setup_wifi(dp.modem) {
        Ok(wifi) => wifi,
        Err(_e) => panic!("Could not connect to wifi")
    };
    println!("wifi connected");
    delay.delay_ms(500);

    // connect to UDP server
    let mut socket = wait_for_udp_connection(delay);
    println!("connected");

    // create data handler
    let mut input = DataHandler::new();

    loop {
        // sample the joysticks
        let trans_mov = joystick1.sample(&mut adc1)?;
        let ang_mov = joystick2.sample(&mut adc1)?;
        input.update(trans_mov, ang_mov);

        // if theres input, send it to server
        if input.input_detected()
        {
            println!("User Input {:?}, {:?}", trans_mov, ang_mov);
            // if connection is broken, wait until reconnect
            if let Err(_) = socket.send(&input.data_as_bytes())
            {
                socket = wait_for_udp_connection(delay);
            }

        }
        delay.delay_ms(100);
    }
}


// -------------------------- wifi --------------------------------------
fn setup_wifi(modem: Modem) -> anyhow::Result<EspWifi<'static>> 
{
    // create a new event loop (configuring wifi happens here I think)
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    // create wifi driver
    let mut wifi: EspWifi = EspWifi::new(modem, sysloop, Some(nvs))?;

    // configure wifi
    wifi.set_configuration(&Configuration::Client(ClientConfiguration 
    {
        ssid: SSID.try_into().unwrap(),
        password: PASSWORD.try_into().unwrap(),
        auth_method: AuthMethod::None,
        ..Default::default()
    }))?;

    // start wifi
    wifi.start()?;

    // connect wifi
    wifi.connect()?;

    // confirm wifi connection
    while ! wifi.is_connected()? || ! wifi.is_up()? 
    {
        println!("Setting up wifi");
        std::thread::sleep(std::time::Duration::from_millis(1000));
    }

    Ok(wifi)
}

// wait until UDP connection and return socket
fn wait_for_udp_connection(delay: Delay) -> UdpSocket
{
    let socket = UdpSocket::bind("0.0.0.0:0").expect("did not bind");

    loop {
        // check to see if connection was established, if not continue loop
        if let Ok(_) = socket.connect(IPADDR) {
            return socket;
        }
        delay.delay_ms(250);
    }
    panic!("Unreachable");
}