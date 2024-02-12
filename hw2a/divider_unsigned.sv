`timescale 1ns / 1ns

// This module performs unsigned division of two 32-bit numbers

module unsigned_divider(
    input wire [31:0] dividend_input,
    input wire [31:0] divisor_input,
    output wire [31:0] quotient_output,
    output wire [31:0] remainder_output
);
    // Internal signals for iterative division process
    wire [31:0] iter_dividend [0:32];
    wire [31:0] iter_quotient [0:32];
    wire [31:0] iter_remainder [0:32];

    // Initialize the first iteration inputs
    assign iter_dividend[0] = dividend_input;
    assign iter_quotient[0] = 32'b0;
    assign iter_remainder[0] = 32'b0;

    generate
    // Creating 32 instances of the iterative division process
    for (genvar i = 0; i < 32; i++) begin : division_step
        divu_iteration step_divide (
            .dividend_in(iter_dividend[i]),
            .divisor_in(divisor_input),
            .remainder_in(iter_remainder[i]),
            .quotient_in(iter_quotient[i]),
            .dividend_out(iter_dividend[i+1]),
            .remainder_out(iter_remainder[i+1]),
            .quotient_out(iter_quotient[i+1])
        );
    end
    endgenerate

    // Assign final quotient and remainder to outputs
    assign remainder_output = iter_remainder[32];
    assign quotient_output = iter_quotient[32];

endmodule

module divu_iteration (
    input wire [31:0] dividend_in,
    input wire [31:0] divisor_in,
    input wire [31:0] remainder_in,
    input wire [31:0] quotient_in,
    output wire [31:0] dividend_out,
    output wire [31:0] remainder_out,
    output wire [31:0] quotient_out
);

    logic [31:0] temp_remainder;
    logic [31:0] next_remainder;
    logic [31:0] updated_quotient;
    logic [31:0] shifted_dividend;

    always_comb begin
        // Prepare for comparison and shifting
        temp_remainder = {remainder_in[30:0], dividend_in[31]};
        shifted_dividend = dividend_in << 1;

        // Conditional update based on comparison
        updated_quotient = (temp_remainder < divisor_in) ? (quotient_in << 1) : ({quotient_in[30:0], 1'b1});
        next_remainder = (temp_remainder < divisor_in) ? temp_remainder : temp_remainder - divisor_in;
    end

    // Set outputs for next iteration
    assign dividend_out = shifted_dividend;
    assign remainder_out = next_remainder;
    assign quotient_out = updated_quotient;

endmodule
