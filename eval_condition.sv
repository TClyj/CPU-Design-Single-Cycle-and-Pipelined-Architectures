module eval_condition (
    input logic [3:0] condition,
    input logic [3:0] flags,
    output logic met
);

    always_comb begin
        met = 1; // Default to true
        case (condition)
            4'b0000: met = (flags[2] == 1); // EQ
            4'b0001: met = (flags[2] == 0); // NE
            4'b1010: met = (flags[3] == flags[0]); // GE
            4'b1100: met = (flags[3] == flags[0]) && (flags[2] == 0); // GT
            4'b1101: met = (flags[3] != flags[0]) || (flags[2] == 1); // LE
            4'b1011: met = (flags[3] != flags[0]); // LT
            default: met = 1;
        endcase
    end

endmodule
