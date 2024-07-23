`timescale 1ns / 1ps

module Full_Subtractor(
    In_A, In_B, Borrow_in, Difference, Borrow_out
    );
    input In_A, In_B, Borrow_in;
    output Difference, Borrow_out;
    wire Dif1, Bor1, Bor2;
    
    // implement full subtractor circuit, your code starts from here.
    // use half subtractor in this module, fulfill I/O ports connection.
    // hint: submodule module_name (
    //          .I/O_port(wire_name),
    //           ...
    //          .I/O_port(wire_name)
    //           );
    Half_Subtractor HSUB1 (
        .In_A(In_A), 
        .In_B(In_B), 
        .Difference(Dif1), 
        .Borrow_out(Bor1)
    );
    Half_Subtractor HSUB2 (
        .In_A(Dif1), 
        .In_B(Borrow_in), 
        .Difference(Difference), 
        .Borrow_out(Bor2)
    );
    or(Borrow_out, Bor1, Bor2);
    
endmodule
