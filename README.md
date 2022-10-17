# Micro:bit Tilt Racer

## Introduction

This is a weekend project to familiarize myself with the embedded Rust ecosystem (such as [RTIC](https://rtic.rs/1/book/en/)) and embedded programming in general. It is not an example of good code.

## Gameplay

There are four game states:
- Start menu, by pressing the "a" button the game starts
- Play mode, where your racer avoids obstacles by skillful tilting of the micro:bit. Every minute the pace quickens
- Explosion animation, which plays after you collide with an obstacle
- A score display, which lights up the LEDs depending on how far you progressed

## Installation

Following the [Embedded Rust Discovery Book](https://docs.rust-embedded.org/discovery/microbit/) and installing the necessary components, you can run the code via:

```
cargo embed --target thumbv7em-none-eabihf
```
