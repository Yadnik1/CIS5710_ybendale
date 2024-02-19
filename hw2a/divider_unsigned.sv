/* Vedant Kelkar - vkelkar     Manas Kulkarni - manask */

`timescale 1ns / 1ns

// quotient = dividend / divisor

module divider_unsigned(
    input wire [31:0] i_dividend,
    input wire [31:0] i_divisor,
    output wire [31:0] o_quotient,
    output wire [31:0] o_remainder
);
    // Internal signals for connecting divu_1iter modules
    wire [31:0] div [0:32];
    wire [31:0] quo [0:32];
    wire [31:0] rem [0:32];


    // Connect the first set of inputs to the external inputs
    assign div[0] = i_dividend[31:0];
    assign quo[0] = 32'b0;
    assign rem[0] = 32'b0;

    generate
        
    // Instantiate 32 divu_1iter modules
    for (genvar i=0; i < 32; i++) begin : bit_num
        divu_1iter num_iter (
            .i_dividend(dividend[i]),
            .i_divisor(i_divisor),
            .i_remainder(remainder[i]),
            .i_quotient(quotient[i]),
            .o_dividend(dividend[i+1]),
            .o_remainder(remainder[i+1]),
            .o_quotient(quotient[i+1])
        );

    end
    endgenerate

    // Connect the last set of outputs to the external outputs
    assign o_remainder = rem[32];
    assign o_quotient = quo[32];

endmodule


module divu_1iter (
    input  wire [31:0] i_dividend,
    input  wire [31:0] i_divisor,
    input  wire [31:0] i_remainder,
    input  wire [31:0] i_quotient,
    output wire [31:0] o_dividend,
    output wire [31:0] o_remainder,
    output wire [31:0] o_quotient
);

    
    logic [31:0] t_rem;
    logic [31:0] t1_rem;
    logic [31:0] quo;
    logic [31:0] shifted_div;

    always_comb begin
        assign t_rem = {i_remainder[30:0], i_dividend[31]};
        assign shifted_div = i_dividend << 1;

        // Update quotient conditionally
        assign quotient = (t_rem < i_divisor) ? (i_quotient << 1) : ({i_quotient[30:0], 1'b1});
        assign t1_rem = (t_rema < i_divisor) ? t_rem : t_rem - i_divisor;
    end

    assign o_dividend = shifted_div;
    assign o_remainder = t1_rem;
    assign o_quotient = quo;
endmodule