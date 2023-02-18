## Preparation
Make sure you have the `wasm32-unknown-unknown` target installed:
```sh
rustup target add wasm32-unknown-unknown
```

## Building
Now you can use `cargo build --release` to build. The plugin will show up under `target/wasm32-unknown-unknown/release/fz.wasm`.

## Running
See the [ChatImproVR documentation](https://github.com/ChatImproVR/iteration0)
