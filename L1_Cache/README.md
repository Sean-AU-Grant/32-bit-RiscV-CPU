This folder contains the L1 Cache SystemVerilog RTL, an RTL test bench for the Cache, and an automated python test bench, as well as 
all associated files.

This Cache is a 4-way set associative cache that supports the RV32I instruction set. 

The test bench and python test script are designed to work cohesively to reduce testing times and function as follows:

1. The test engineer should create set of instructions similar to the Cache Test Script.xlsx, and then covert those instructions to micro_insts.csv (do not include the header)
2. Once this file has been created, the python test script can be run for the first time.
3. On the first run, the python script will convert the CSV into the neccessary hex file "mem_test.hex" which is formatted to be read and run by the SystemVerilog test bench
4. The outputs of this first run are only for creating the .hex file
5. Next, run the SystemVerilog test bench, which will run all of the instructions in the "mem_test.hex" file, and save all load instrucitons, i.e instructions that produce verifyable outputs, into "sim_results.csv"
6. Finally, rerun the python test script. Now, in addition to creating the .hex file, the python code will run the instructions on a "golden model" that is not timing accurate, and store all outputs to a list internally.
7. The python outputs will then be compared to the outputs of test bench, "sim_results.csv", and if any instructions are not found, duplicate, or found to have the wrong data, the user is alerted in the terminal.

The terminal output can then be used to determine which instrucitons are failing, allowing for quicker debug turnaround.
