# Embedded_Synth

ST NUCLEO-L432KC microcontroller module that contains a STM32L432KCU6U processor, which has an Arm Cortex-M4 core.

Implemented a synth using a couple button, knobs, piezo speaker, and a small microcontroller. 
Can make its own waveforms like Sawtooth and Sine.
Implements different effects like Echo & Reverb. Allows you to change the octave and the volume. 

Implements threading and very heavy use of thread-safe operations like semaphores, mutex, and atomic instructions.

Knobs produce quadrature waves when turned. Programmed state checkers to best determine which direction and how much the knob:s was/were turned.
Can communicate on serial via UART. Both input and output.

The best feature is that it plays Frère Jaques if you press the right-most knob.

Coded in Arduino IDE using C and C++.

Embedded systems design.
