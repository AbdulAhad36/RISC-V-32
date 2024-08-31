module mux4_1(y,in1,in2,in3,in4,s1,s2);
input in1,in2,in3,in4;
input s1,s2;
output reg y;

always@ (s1 or s2 or in2 or in3 or in4)
begin
if(s1==1'b0 & s2==1'b0)
y=in1;
else if(s1==1'b0 & s2==1'b1)
y=in2;
else if(s1==1'b1 & s2==1'b0)
y=in3;
else if(s1==1'b1 & s2==1'b1)
y=in4;
end
endmodule

