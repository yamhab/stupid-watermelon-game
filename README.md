# stupid-watermelon-game

*A Rust implementation of Suika Game*

![Video demo of the game](demo.gif)

---

Remember that awful game about dropping and merging fruits in a container that went crazy some time
ago? I implemented a very simple and hastily-made version of it in Rust here. It uses Macroquad
for windowing, rendering, and input, and Rapier for the physics engine. The fruits are just colored
circles, and there's a basic scoring system. Currently, the game window just closes if you lose.
