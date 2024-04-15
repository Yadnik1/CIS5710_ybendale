ls
cd hw2a
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
pytest-3 testbench_1iter.py
clear
pytest-3 testbench_1iter.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench_1iter.py
pytest-3 testbench.py
ls
cd hw2b
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench_gp4.py
exit
cd hw2b
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
exit
cd hw2b
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
exit
testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
cd hw2b
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
exit
ls
cd hw2b
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench_4.py
pytest-3 testbench_4.py
pytest-3 testbench_gp4.py
exit
ls
cd hw2b
pytest-3 testbench_gp4.py
cd hw2b
module gp4(
    input wire [3:0] gin, pin,
    input wire cin,
    output wire gout, pout,
    output wire [2:0] cout
);
    // Intermediate propagate signals for carry computation
    wire [3:0] p_intermediate;
    // Compute intermediate propagate signals
    assign p_intermediate[0] = pin[0];
    assign p_intermediate[1] = pin[1] & pin[0];
    assign p_intermediate[2] = pin[2] & pin[1] & pin[0];
    assign p_intermediate[3] = pin[3] & pin[2] & pin[1] & pin[0];
    // Compute carry outs
    assign cout[0] = gin[0] | (pin[0] & cin);
    assign cout[1] = gin[1] | (pin[1] & cout[0]);
    assign cout[2] = gin[2] | (pin[2] & cout[1]);
    // Compute aggregate generate and propagate signals
    assign gout = gin[3] | (pin[3] & gin[2]) | (p_intermediate[2] & gin[1]) | (p_intermediate[3] & gin[0]);
    assign pout 
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
exit
cd hw2b
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 codecheck.py
pytest-3 codecheck.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 codecheck.py
pytest-3 codecheck.py
pytest-3 codecheck.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench_gp4.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
ls
cd hw2b
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
pytest-3 testbench.py
RVTEST_ALUBR=1 pytest-3 -s testbench.py
ls
cd hw3-singlecycle/
RVTEST_ALUBR=1 pytest-3 -s testbench.py
RVTEST_ALUBR=1 pytest-3 -s testbench.py
RVTEST_ALUBR=1 pytest-3 -s testbench.py
RVTEST_ALUBR=1 pytest-3 -s testbench.py
RVTEST_ALUBR=1 pytest-3 -s testbench.py
exit
