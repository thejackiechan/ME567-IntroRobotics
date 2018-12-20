//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

    // STENCIL: reference quaternion code has the following functions:
    //   quaternion_from_axisangle
    //   quaternion_normalize
    //   quaternion_to_rotation_matrix
    //   quaternion_multiply

    function quaternion_from_axisangle(axis, angle){ // axis = u = [ux uy uz]

    	var ux = axis[0];
    	var uy = axis[1];
    	var uz = axis[2];

    	var q = [Math.cos(angle/2), ux*Math.sin(angle/2), uy*Math.sin(angle/2), uz*Math.sin(angle/2)];

    	return q;
    }

    function quaternion_normalize(q){

    	var i;
    	var a = q[0];
    	var b = q[1];
    	var c = q[2];
    	var d = q[3];

    	var scale = Math.pow((Math.pow(a,2) + Math.pow(b,2) + Math.pow(c,2) + Math.pow(d,2)), 0.5);

    	for(i = 0; i < q.length; i++){
    		q[i] = q[i]/scale;
    	}
    	return q;
    }


    function quaternion_to_rotation_matrix(q){ // q = [q0 q1 q2 q3]

    	var q0 = q[0];
    	var q1 = q[1];
    	var q2 = q[2];
    	var q3 = q[3];

    	var mat = [[Math.pow(q0,2) + Math.pow(q1,2) - Math.pow(q2,2) - Math.pow(q3,2), 2*(q1*q2 - q0*q3), 2*(q0*q2 + q1*q3), 0],
    				[2*(q1*q2 + q0*q3), Math.pow(q0,2) - Math.pow(q1,2) + Math.pow(q2,2) - Math.pow(q3,2), 2*(q2*q3 - q0*q1), 0],
    				[2*(q1*q3 - q0*q2), 2*(q0*q1 + q2*q3), Math.pow(q0,2) - Math.pow(q1,2) - Math.pow(q2,2) + Math.pow(q3,2), 0],
                    [0, 0, 0, 1]];

    	return mat;
    }


    function quaternion_multiply(q1,q2){

    	var a = q1[0];
    	var b = q1[1];
    	var c = q1[2];
    	var d = q1[3];

    	var e = q2[0];
    	var f = q2[1];
    	var g = q2[2];
    	var h = q2[3];

    	var q_0 = a*e - b*f - c*g - d*h;
    	var q_1 = a*f + b*e + c*h - d*g;
    	var q_2 = a*g - b*h + c*e + d*f;
    	var q_3 = a*h + b*g - c*f + d*e;

    	var q = [q_0, q_1, q_2, q_3];

    	return q;
    }







