
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }
    kineval.buildFKTransforms(); 
}

kineval.buildFKTransforms = function buildFKTransforms(){ 

    for(x in robot.joints){
        robot.joints[x].name = x;
        robot.joints[x].visited = false; // reset to false for next iteration
    }

    mat_stack = []; 
    mat_stack.push(generate_identity(4));

    traverseFKBase();
    traverseFKLink(robot.base);
}

function traverseFKBase(q){ // default collision behavior checks base

    if(typeof q == 'undefined'){
        var heading = [[0],[0],[1],[1]];
        var lateral = [[1],[0],[0],[1]];
        
        robot.origin.xform = DR(robot.origin.xyz,robot.origin.rpy,false);
            
        if(typeof robot.links_geom_imported != "undefined" && robot.links_geom_imported == true){
            robot.links[robot.base].xform = DR(robot.origin.xyz,robot.origin.rpy,true); // ROS robot
        }else{
            robot.links[robot.base].xform = robot.origin.xform // non-ROS robot
        }    
        mat_stack.push(robot.links[robot.base].xform);

        robot_heading = matrix_multiply(robot.origin.xform,heading); // global var
        robot_lateral = matrix_multiply(robot.origin.xform,lateral);
    }else{
        for(x in robot.joints){ // reset joints
            robot.joints[x].name = x;
            robot.joints[x].visited = false; // reset to false for next iteration
        }
       
        q_stack = [generate_identity(4)]; // initialize separate stack keeping track of configurations

        var q_xyz = [q[0],q[1],q[2]];
        var q_rpy = [q[3],q[4],q[5]];

        if(typeof robot.links_geom_imported != "undefined" && robot.links_geom_imported == true){
            robot.links[robot.base].q_xform = DR(q_xyz,q_rpy,true);
        }else{
            robot.links[robot.base].q_xform = DR(q_xyz,q_rpy,false);
        }
        q_stack.push(robot.links[robot.base].q_xform);
    }
}

function traverseFKLink(link,q){ // takes in link name 

    var l = robot.links[link];
    var child;

    if(typeof q == "undefined"){
        if(l.children == undefined){ // base case if link is leaf
            l.xform = mat_stack.pop();
            child = l.parent;
        }else if(robot.joints[l.children[l.children.length - 1]].visited == true && link != robot.base){ // move up hierarchy if link isn't base and all children have been visited
            mat_stack.pop();
            child = l.parent;
        }else{
            l.xform = mat_stack[mat_stack.length - 1];
            for(i = 0;i < l.children.length;i++){
                if(robot.joints[l.children[i]].visited == false){
                    child = l.children[i];
                    break;
                }
            }
        }
        if(link == robot.base && robot.joints[l.children[l.children.length - 1]].visited == true){ // exit if link is base and all base's children have been visited 
            mat_stack.pop();
            mat_stack.pop();
        }else{ // should enter here after any of the first three cases
            traverseFKLink(traverseFKJoint(child));
        }
    }else{
        if(l.children == undefined){ // base case if link is leaf
            l.q_xform = q_stack.pop();
            child = l.parent;
        }else if(robot.joints[l.children[l.children.length - 1]].visited == true && link != robot.base){ // move up hierarchy if link isn't base and all children have been visited
            q_stack.pop();
            child = l.parent;
        }else{
            l.q_xform = q_stack[q_stack.length - 1];
            for(i = 0;i < l.children.length;i++){
                if(robot.joints[l.children[i]].visited == false){
                    child = l.children[i];
                    break;
                }
            }
        }
        if(link == robot.base && robot.joints[l.children[l.children.length - 1]].visited == true){ // exit if link is base and all base's children have been visited 
            q_stack.pop();
            q_stack.pop();
        }else{ // should enter here after any of the first three cases
            traverseFKLink(traverseFKJoint(child,q),q);
        }
    }
}

function traverseFKJoint(joint,q){ // helper function that takes in joints and returns links

    var j = robot.joints[joint];

    if(j.visited == true){
        return j.parent;
    }else{
        if(typeof q == "undefined"){
            var mat = DR(j.origin.xyz,j.origin.rpy,false); // compute matrix for the joint
            j.xform = matrix_multiply(mat_stack[mat_stack.length - 1],mat); // computes the transformation matrix

            if(typeof j.type == "undefined" || j.type == "revolute" || j.type == "continuous"){
                var quat = quaternion_from_axisangle(j.axis,j.angle); 
                var norm_q = quaternion_normalize(quat);
                var joint_rot = quaternion_to_rotation_matrix(norm_q);

                j.xform = matrix_multiply(j.xform, joint_rot);

            }else if(j.type == "prismatic"){
                var norm_axis = vector_normalize(j.axis);
                var scaled_axis = scaleByAngle(j.angle,norm_axis);
                var joint_trans_mat = generate_translation_matrix(scaled_axis[0], scaled_axis[1], scaled_axis[2]);

                j.xform = matrix_multiply(j.xform, joint_trans_mat);
            }
            mat_stack.push(j.xform);
        }else{
            var mat = DR(j.origin.xyz,j.origin.rpy,false);
            j.q_xform = matrix_multiply(q_stack[q_stack.length - 1],mat);

            if(typeof j.type == "undefined" || j.type == "revolute" || j.type == "continuous"){
                var q_quat = quaternion_from_axisangle(j.axis, q[q_names[joint]]); // q_robot_config should be global var
                var q_norm_q = quaternion_normalize(q_quat);
                var q_joint_rot = quaternion_to_rotation_matrix(q_norm_q);

                j.q_xform = matrix_multiply(j.q_xform, q_joint_rot);

            }else if(j.type == "prismatic"){
                var norm_axis = vector_normalize(j.axis);
                var q_scaled_axis = scaleByAngle(q[q_names[joint]],norm_axis); // again q_robot_config should be global
                var q_joint_trans_mat = generate_translation_matrix(q_scaled_axis[0], q_scaled_axis[1], q_scaled_axis[2]);

                j.q_xform = matrix_multiply(j.q_xform, q_joint_trans_mat);   
            }
            q_stack.push(j.q_xform);
        }
        j.visited = true;
        return j.child;
    }
}

function DR(xyz,rpy,offset){ // takes in two arrays .xyz and .rpy

    var dist = generate_translation_matrix(xyz[0],xyz[1],xyz[2]);

    var roll = generate_rotation_matrix_X(rpy[0]); 
    var pitch = generate_rotation_matrix_Y(rpy[1]);
    var yaw = generate_rotation_matrix_Z(rpy[2]);
    
    var m1 = matrix_multiply(dist,yaw);
    var m2 = matrix_multiply(m1,pitch);
    var m3 = matrix_multiply(m2,roll);

    if(offset == true){
        var rX = generate_rotation_matrix_X(-Math.PI/2); 
        var rZ = generate_rotation_matrix_Z(-Math.PI/2); 
        var ROSto3js = matrix_multiply(rX,rZ);
        var T = matrix_multiply(m3,ROSto3js);
        return T;
    }else{
        return m3;
    }
}

    // STENCIL: reference code alternates recursive traversal over 
    //   links and joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // user interface needs the heading (z-axis) and lateral (x-axis) directions
    //   of robot base in world coordinates stored as 4x1 matrices in
    //   global variables "robot_heading" and "robot_lateral"
    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

