function x = worldCoord(leftUV, b, rightUV, c)

A = [b(1)-(b(9)*leftUV(1)), b(2)-(b(10)*leftUV(1)), b(3)-(b(11)*leftUV(1));
    b(5)-(b(9)*leftUV(2)), b(6)-(b(10)*leftUV(2)), b(7)-(b(11)*leftUV(2));
    c(1)-(c(9)*rightUV(1)), c(2)-(c(10)*rightUV(1)), c(3)-(c(11)*rightUV(1));
    c(5)-(c(9)*rightUV(2)), c(6)-(c(10)*rightUV(2)), c(7)-(c(11)*rightUV(2));
    ];
B = [leftUV(1) - b(4);
    leftUV(2) - b(8);
    rightUV(1) - c(4);
    rightUV(2) - c(8)
    ];
x = pinv(A)*B;

end