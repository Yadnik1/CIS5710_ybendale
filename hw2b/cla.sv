/* Yadnik Bendale: ybendale Rajneesh Kumar: rajneesh */

`timescale 1ns / 1ps

module gp1(input wire a, b, output wire g, p);
   assign g = a & b;
   assign p = a | b;
endmodule

module gp4(input wire [3:0] gin, pin, input wire cin, output wire gout, pout, output wire [2:0] cout);
   assign cout[0] = gin[0] | (pin[0] & cin);
   assign cout[1] = gin[1] | (pin[1] & gin[0]) | (pin[1] & pin[0] & cin);
   assign cout[2] = gin[2] | (pin[2] & gin[1]) | (pin[2] & pin[1] & gin[0]) | (pin[2] & pin[1] & pin[0] & cin);
   assign gout = gin[3] | (pin[3] & gin[2]) | (pin[3] & pin[2] & gin[1]) | (pin[3] & pin[2] & pin[1] & gin[0]);
   assign pout = pin[3] & pin[2] & pin[1] & pin[0];
endmodule

module gp8(input wire [7:0] gin, pin, input wire cin, output wire gout, pout, output wire [6:0] cout);
   logic [6:0] carry_out_temp;
   always_comb begin
      carry_out_temp[0] = gin[0] | (pin[0] & cin);
      carry_out_temp[1] = gin[1] | (pin[1] & carry_out_temp[0]);
      carry_out_temp[2] = gin[2] | (pin[2] & carry_out_temp[1]);
      carry_out_temp[3] = gin[3] | (pin[3] & carry_out_temp[2]);
      carry_out_temp[4] = gin[4] | (pin[4] & carry_out_temp[3]);
      carry_out_temp[5] = gin[5] | (pin[5] & carry_out_temp[4]);
      carry_out_temp[6] = gin[6] | (pin[6] & carry_out_temp[5]);
   end
   assign gout = gin[7] | (pin[7] & carry_out_temp[6]);
   assign pout = (& pin);
   assign cout = carry_out_temp;
endmodule

module cla(input wire [31:0] a, b, input wire cin, output wire [31:0] sum);
   wire [31:0] gin_a, pin_a;
   reg [31:0] inter_sum;
   wire [30:0] cout; 
   wire [4:0] g_out, p_out;
   generate
      for(genvar i = 0; i < 32; i = i +1) begin : gp_a
         gp1 gp_a_(.a(a[i]), .b(b[i]), .g(gin_a[i]), .p(pin_a[i]));
      end
   endgenerate
   gp8 gp8_a(.gin(gin_a[7:0]), .pin(pin_a[7:0]), .cin(cin), .gout(g_out[0]), .pout(p_out[0]), .cout(cout[6:0]));
   for(genvar j = 1; j < 4; j = j + 1) begin : gp8_b
      gp8 gp8_b_(.gin(gin_a[(j+1)*7:j*7]), .pin(pin_a[(j+1)*7:j*7]), .cin(cout[(j*7)-1]), .gout(g_out[j]), .pout(p_out[j]), .cout(cout[((j+1)*7)-1:j*7]));
   end
   gp4 gp8_final(.gin(gin_a[31:28]), .pin(pin_a[31:28]), .cin(cout[27]), .gout(g_out[4]), .pout(p_out[4]), .cout(cout[30:28])); 
   always_comb begin
      for(integer k = 0; k < 32; k = k + 1) begin
         if(k == 0) begin
            inter_sum[k] = a[k] ^ b[k] ^ cin;
         end else begin
            inter_sum[k] = a[k] ^ b[k] ^ cout[k-1];
         end
      end
   end   
   assign sum = inter_sum;

endmodule
