
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // get endeffector Cartesian position in the world
   endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

   // compute distance of endeffector to target
   kineval.params.trial_ik_random.distance_current = Math.sqrt(
           Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
           + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
           + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

   // if target reached, increment scoring and generate new target location
   // KE 2 : convert hardcoded constants into proper parameters
   if (kineval.params.trial_ik_random.distance_current < 0.01) {
       kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
       kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
       kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
       kineval.params.trial_ik_random.targets += 1;
       textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
   }
}

// endeffector_target_world is an object with two fields with the target endeffector position (3D homogeneous vector so 4 x 1) and orientation in the world frame 
//(Euler angles)
// endeffector_joint is the name of the joint directly connected to endeffector
// endeffector_position_local is the location of endeffector in the local joint frame

// Steps for IK
// 1) specify target location: given by endeffector_target_world
// 2) Form kinematic chain (endeffector joint all the way back to joint whose parent is base link)
//    a) transform endeffector into world
// 3) Iterate over chain to update robot.controls
//    a) build Jacobian and compute Jacobian Transpose and Pseudoinverse
// 	  b) apply updated controls to each joint (matrix inversion provided by numeric.inv(J))

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration
    var i;
    chain = [];

    // Build kinematic chain
    obtain_parent_Link(endeffector_joint,endeffector_joint); // builds kinematic chain
 
    var endeffector_position_global = matrix_multiply(robot.joints[endeffector_joint].xform,endeffector_position_local); // does this transform endeffector into world frame?
    endeffector_position_global.pop(); // makes it 3x1 column

    // Traverse kinematic chain and build Jacobian
    var J = traverseChain(endeffector_position_global); // traverses chain and builds Jacobian

    // Build delta_x
    var delta_x = calc_delta_x(endeffector_target_world, endeffector_joint, endeffector_position_global);

    // Multiply Jacobian Transpose or PseudoInverse
	if(!kineval.params.ik_pseudoinverse){ // Jacobian Transpose
		var delta_q = matrix_multiply(matrix_transpose(J),delta_x); // Nx1 large column
    }else{ 
    	var delta_q = Jacobian_Pseudoinverse(J,delta_x);
    }
    var delta_angle = scaleByAngle(kineval.params.ik_steplength,delta_q); // 1xN row vec

    // Update robot controls
    for(i = 0;i < chain.length;i++){  
    	var j = robot.joints[chain[i]];
    	j.control = delta_angle[i]; 
    }
}

// Helper functions for IK

function obtain_parent_Link(joint, endeffector_joint){

	var j = robot.joints[joint];

	if(j.parent == robot.base){
		if(robot.joints[endeffector_joint].type != "fixed"){
			chain.push(endeffector_joint);
		}
	}else{
		obtain_parent_Link(obtain_parent_Joint(j.parent), endeffector_joint);
	}
}

function obtain_parent_Joint(link){

	var l = robot.links[link];

	if(l.parent.type != "fixed"){
		chain.unshift(l.parent); // adds to front of chain
	}	
	return l.parent; // returns parent joint
}

function extract_Angle(mat){ // assumes rot XYZ, seems to work

	var thetaX = Math.atan2(-mat[1][2],mat[2][2]);
	var thetaY = Math.asin(mat[0][2]);
	var thetaZ = Math.atan2(-mat[0][1],mat[0][0]);

	return matrix_transpose([thetaX,thetaY,thetaZ]);
}

function Jacobian_Pseudoinverse(J,delta_x){
	var pseudomat = matrix_pseudoinverse(J);
    var delta_q = matrix_multiply(pseudomat,delta_x);

    return delta_q;
}

function jointAxis(j){

    var hom_axis = matrix_transpose(j.axis);  // parent?
    hom_axis.push([1]);
    var joint_axis = matrix_multiply(j.xform, hom_axis); // column vec
   	joint_axis.pop();

   	return joint_axis;
}

function jointOrigin(j){

	var hom_origin = matrix_transpose(j.origin.xyz); // column vec
    hom_origin.push([1]);
    var joint_origin_xyz = matrix_multiply(robot.links[j.parent].xform, hom_origin); // column vec
   	joint_origin_xyz.pop(); // now have 3x1 column vecs

   	return joint_origin_xyz;
}

function calc_delta_x(endeffector_target_world, endeffector_joint, endeffector_position_global){

	var x_des = matrix_copy(endeffector_target_world.position);
    x_des.pop(); // makes 3x1
    var orientation_des = matrix_transpose(endeffector_target_world.orientation); // 3x1
    var x_n = matrix_copy(endeffector_position_global);
    var x_n_orientation = extract_Angle(robot.joints[endeffector_joint].xform); // obtain Euler angles as 3x1

    for(i = 0;i < endeffector_target_world.orientation.length;i++){
    	if(!kineval.params.ik_orientation_included){ // if no orientation
    		x_des.push([0]); // 6x1
    		x_n.push([0]);
    	}else{
    		x_des.push(orientation_des[i]);  // if orientation is included
    		x_n.push(x_n_orientation[i]);  // push Euler angles
    	}
    }

    var delta_x = vector_sum(matrix_transpose(x_des),matrix_transpose(x_n),false); // row vec 1x6
    delta_x = matrix_transpose(delta_x); // column 6x1

    return delta_x;
}

function traverseChain(endeffector_position_global){

	var J = [];
    var J_ang_stack = [];

	for(i = 0;i < chain.length;i++){
    	var j = robot.joints[chain[i]];

    	var joint_axis = jointAxis(j); // check if functions changed things
    	var joint_origin_xyz = jointOrigin(j);

    	var axis_world = vector_sum(matrix_transpose(joint_axis), matrix_transpose(joint_origin_xyz), false); // vector_sum takes in row vecs, this is also equal to J_ang for 
    	//rot joints and J_lin for prismatic
    	var endeffector_world = vector_sum(matrix_transpose(endeffector_position_global), matrix_transpose(joint_origin_xyz), false);
    	var J_lin = vector_cross(axis_world,endeffector_world); // returns column vec

    	if(j.type != "undefined" && j.type === "prismatic"){
    		J.push(matrix_transpose(axis_world)); // axis_world is a column
    		J_ang_stack.push([[0],[0],[0]]); // if prismatic joint is prismatic, there are no angular entries
    	}else{
    		J.push(J_lin); // for rotational joints, pushes 3x1
    		J_ang_stack.push(matrix_transpose(axis_world));
    	}		
    }

    J = matrix_transpose(J); // 3xN
    J_ang_stack = matrix_transpose(J_ang_stack); // 3xN

    for(i = 0;i < J_ang_stack.length;i++){ // concatenate to form 6xN Jacobian 
    	J.push(J_ang_stack[i]);
    }
    return J;
}
