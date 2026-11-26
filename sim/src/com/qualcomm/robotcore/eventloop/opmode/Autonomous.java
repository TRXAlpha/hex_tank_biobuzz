package com.qualcomm.robotcore.eventloop.opmode; public @interface Autonomous { String name(); String group() default ""; String preselectTeleOp() default ""; }
