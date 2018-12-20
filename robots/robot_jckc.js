//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name 
robot.name = "scorpion"; // modeled after crawler

// initialize start pose of robot in the world
robot.origin = {xyz: [0,1,0], rpy:[0,0,0]};  // held a bit over the ground plane

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";  

        
// specify and create data objects for the links of the robot
robot.links = {
    "base": {},  
    "leg1_upper": {}, 
    "leg1_middle": {}, 
    "leg1_lower": {}, 
    "leg2_upper": {}, 
    "leg2_middle": {}, 
    "leg2_lower": {}, 
    "leg3_upper": {}, 
    "leg3_middle": {}, 
    "leg3_lower": {}, 
    "leg4_upper": {}, 
    "leg4_middle": {}, 
    "leg4_lower": {}, 
    "leg5_upper": {}, 
    "leg5_middle": {}, 
    "leg5_lower": {}, 
    "leg6_upper": {}, 
    "leg6_middle": {}, 
    "leg6_lower": {}, 
    "leg7_upper": {}, 
    "leg7_middle": {}, 
    "leg7_lower": {}, 
    "leg8_upper": {}, 
    "leg8_middle": {}, 
    "leg8_lower": {}, 
    "tail_lower": {}, 
    "tail_middle" : {},
    "tail_upper" : {},
    "tail_spike" : {},
    "claw1_upper" : {},
    "claw1_middle" : {},
    "claw1_lower" : {},
    "claw1_pincer1" : {},
    "claw1_pincer2" : {},

    "claw2_upper" : {},
    "claw2_middle" : {},
    "claw2_lower" : {},
    "claw2_pincer1" : {},
    "claw2_pincer2" : {}

};

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "leg1_ankle";
robot.endeffector.position = [[0],[0],[0.9],[1]]

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

/*      joint definition template
        // specify parent/inboard link and child/outboard link
        robot.joints.joint1 = {parent:"link1", child:"link2"};
        // joint origin's offset transform from parent link origin
        robot.joints.joint1.origin = {xyz: [5,3,0], rpy:[0,0,0]}; 
        // joint rotation axis
        robot.joints.joint1.axis = [0.0,0.0,1.0]; 
*/


// roll-pitch-yaw defined by ROS as corresponding to x-y-z 
//http://wiki.ros.org/urdf/Tutorials/Create%20your%20own%20urdf%20file

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.leg1_hip = {parent:"base", child:"leg1_upper"};
robot.joints.leg1_hip.origin = {xyz: [0.3,0.0,0.9], rpy:[0,Math.PI/2,0]};
robot.joints.leg1_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg1_knee = {parent:"leg1_upper", child:"leg1_middle"};
robot.joints.leg1_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg1_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg1_ankle = {parent:"leg1_middle", child:"leg1_lower"};
robot.joints.leg1_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg1_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg2_hip = {parent:"base", child:"leg2_upper"};
robot.joints.leg2_hip.origin = {xyz: [0.3,0.0,-0.9], rpy:[0,Math.PI/2,0]};
robot.joints.leg2_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg2_knee = {parent:"leg2_upper", child:"leg2_middle"};
robot.joints.leg2_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg2_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg2_ankle = {parent:"leg2_middle", child:"leg2_lower"};
robot.joints.leg2_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg2_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg3_hip = {parent:"base", child:"leg3_upper"};
robot.joints.leg3_hip.origin = {xyz: [-0.3,0.0,0.9], rpy:[0,-Math.PI/2,0]};
robot.joints.leg3_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg3_knee = {parent:"leg3_upper", child:"leg3_middle"};
robot.joints.leg3_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg3_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg3_ankle = {parent:"leg3_middle", child:"leg3_lower"};
robot.joints.leg3_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg3_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg4_hip = {parent:"base", child:"leg4_upper"};
robot.joints.leg4_hip.origin = {xyz: [-0.3,0.0,-0.9], rpy:[0,-Math.PI/2,0]};
robot.joints.leg4_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg4_knee = {parent:"leg4_upper", child:"leg4_middle"};
robot.joints.leg4_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg4_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg4_ankle = {parent:"leg4_middle", child:"leg4_lower"};
robot.joints.leg4_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg4_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg5_hip = {parent:"base", child:"leg5_upper"};
robot.joints.leg5_hip.origin = {xyz: [0.3,0.0,0.3], rpy:[0,Math.PI/2,0]};
robot.joints.leg5_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg5_knee = {parent:"leg5_upper", child:"leg5_middle"};
robot.joints.leg5_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg5_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg5_ankle = {parent:"leg5_middle", child:"leg5_lower"};
robot.joints.leg5_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg5_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg6_hip = {parent:"base", child:"leg6_upper"};
robot.joints.leg6_hip.origin = {xyz: [0.3,0.0,-0.3], rpy:[0,Math.PI/2,0]};
robot.joints.leg6_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg6_knee = {parent:"leg6_upper", child:"leg6_middle"};
robot.joints.leg6_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg6_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg6_ankle = {parent:"leg6_middle", child:"leg6_lower"};
robot.joints.leg6_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg6_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg7_hip = {parent:"base", child:"leg7_upper"};
robot.joints.leg7_hip.origin = {xyz: [-0.3,0.0,0.3], rpy:[0,-Math.PI/2,0]};
robot.joints.leg7_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg7_knee = {parent:"leg7_upper", child:"leg7_middle"};
robot.joints.leg7_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg7_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg7_ankle = {parent:"leg7_middle", child:"leg7_lower"};
robot.joints.leg7_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg7_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.leg8_hip = {parent:"base", child:"leg8_upper"};
robot.joints.leg8_hip.origin = {xyz: [-0.3,0.0,-0.3], rpy:[0,-Math.PI/2,0]};
robot.joints.leg8_hip.axis = [0.0,1.0,0.0]; 

robot.joints.leg8_knee = {parent:"leg8_upper", child:"leg8_middle"};
robot.joints.leg8_knee.origin = {xyz: [0.0,0.0,0.4], rpy:[-Math.PI/4,0,0]};
robot.joints.leg8_knee.axis = [1.0,0.0,0.0]; 

robot.joints.leg8_ankle = {parent:"leg8_middle", child:"leg8_lower"};
robot.joints.leg8_ankle.origin = {xyz: [0.0,0.0,0.6], rpy:[Math.PI/2,0,0]};
robot.joints.leg8_ankle.axis = [1.0,0.0,0.0]; 

robot.joints.tail_piece1 = {parent:"base", child:"tail_lower"};
robot.joints.tail_piece1.origin = {xyz: [0.0,0.4,-1.3], rpy:[-Math.PI/8,Math.PI/4,0]}; 
robot.joints.tail_piece1.axis = [0.0,1.0,0.0];

robot.joints.tail_piece2 = {parent:"tail_lower", child:"tail_middle"};
robot.joints.tail_piece2.origin = {xyz: [0.0,0.7,0.2], rpy:[Math.PI/4,0,0]}; 
robot.joints.tail_piece2.axis = [0.0,1.0,0.0];

robot.joints.tail_piece3 = {parent:"tail_middle", child:"tail_upper"};
robot.joints.tail_piece3.origin = {xyz: [-0.2,0.45,0], rpy:[0,0,Math.PI/3]}; 
robot.joints.tail_piece3.axis = [0.0,1.0,0.0];

robot.joints.tail_piece4 = {parent:"tail_upper", child:"tail_spike"};
robot.joints.tail_piece4.origin = {xyz: [0.0,0.6,0.0], rpy:[0,0,0]}; 
robot.joints.tail_piece4.axis = [0.0,1.0,0.0];

robot.joints.claw1_piece1 = {parent:"base", child:"claw1_upper"};
robot.joints.claw1_piece1.origin = {xyz: [-0.55,0.1,1.35], rpy:[Math.PI/3,Math.PI/6,Math.PI/2]};
robot.joints.claw1_piece1.axis = [0.0,1.0,0.0];

robot.joints.claw1_piece2 = {parent:"claw1_upper", child:"claw1_middle"};
robot.joints.claw1_piece2.origin = {xyz: [-0.1,0.3,0], rpy:[0,0,Math.PI/4]};
robot.joints.claw1_piece2.axis = [0.0,1.0,0.0];

robot.joints.claw1_piece3 = {parent:"claw1_middle", child:"claw1_lower"};
robot.joints.claw1_piece3.origin = {xyz: [0.0,0.3,0.1], rpy:[Math.PI/3,0,0]};
robot.joints.claw1_piece3.axis = [0.0,1.0,0.0];

robot.joints.pincer1_piece1 = {parent:"claw1_lower", child:"claw1_pincer1"};
robot.joints.pincer1_piece1.origin = {xyz: [0.0,0.4,0.05], rpy:[0,-Math.PI/3,-Math.PI/6]}; 
robot.joints.pincer1_piece1.axis = [0.0,1.0,0.0];

robot.joints.pincer1_piece2 = {parent:"claw1_pincer1", child:"claw1_pincer2"};
robot.joints.pincer1_piece2.origin = {xyz: [0.0,0.1,-0.2], rpy:[0,0,0]}; 
robot.joints.pincer1_piece2.axis = [0.0,1.0,0.0];

robot.joints.claw2_piece1 = {parent:"base", child:"claw2_upper"};
robot.joints.claw2_piece1.origin = {xyz: [0.55,0.05,1.35], rpy:[Math.PI/3,0,-Math.PI/2]};  
robot.joints.claw2_piece1.axis = [0.0,1.0,0.0];

robot.joints.claw2_piece2 = {parent:"claw2_upper", child:"claw2_middle"};
robot.joints.claw2_piece2.origin = {xyz: [-0.15,0.4,0], rpy:[0,0,Math.PI/4]};
robot.joints.claw2_piece2.axis = [0.0,1.0,0.0];

robot.joints.claw2_piece3 = {parent:"claw2_middle", child:"claw2_lower"};
robot.joints.claw2_piece3.origin = {xyz: [-0.05,0.4,0], rpy:[0,Math.PI/4,Math.PI/6]};
robot.joints.claw2_piece3.axis = [0.0,1.0,0.0];

robot.joints.pincer2_piece1 = {parent:"claw2_lower", child:"claw2_pincer1"};
robot.joints.pincer2_piece1.origin = {xyz: [-0.2,0.3,0], rpy:[0,Math.PI/3,Math.PI/6]}; 
robot.joints.pincer2_piece1.axis = [0.0,1.0,0.0];

robot.joints.pincer2_piece2 = {parent:"claw2_pincer1", child:"claw2_pincer2"};
robot.joints.pincer2_piece2.origin = {xyz: [0.0,-0.05,0.2], rpy:[0,Math.PI/3,0]}; 
robot.joints.pincer2_piece2.axis = [0.0,1.0,0.0];


//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

/*  threejs geometry definition template, will be used by THREE.Mesh() to create threejs object
    // create threejs geometry and insert into links_geom data object
    links_geom["link1"] = new THREE.CubeGeometry( 5+2, 2, 2 );

    // example of translating geometry (in object space)
    links_geom["link1"].applyMatrix( new THREE.Matrix4().makeTranslation(5/2, 0, 0) );

    // example of rotating geometry 45 degrees about y-axis (in object space)
    var temp3axis = new THREE.Vector3(0,1,0);
    links_geom["link1"].rotateOnAxis(temp3axis,Math.PI/4);
*/

// define threejs geometries and associate with robot links 
links_geom = {};

links_geom["base"] = new THREE.CubeGeometry( 1, 0.4, 2.3 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["leg1_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg1_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg1_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg1_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg1_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg1_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg2_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg2_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg2_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg2_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg2_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg2_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg3_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg3_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg3_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg3_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg3_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg3_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg4_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg4_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg4_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg4_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg4_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg4_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg5_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg5_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg5_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg5_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg5_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg5_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg6_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg6_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg6_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg6_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg6_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg6_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg7_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg7_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg7_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg7_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg7_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg7_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["leg8_upper"] = new THREE.CubeGeometry( 0.3, 0.3, 0.3 );
links_geom["leg8_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.15) );

links_geom["leg8_middle"] = new THREE.CubeGeometry( 0.3, 0.3, 0.6 );
links_geom["leg8_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.3) );

links_geom["leg8_lower"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["leg8_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["tail_lower"] = new THREE.CylinderGeometry( 0.2, 0.1, 1.0 );
links_geom["tail_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["tail_middle"] = new THREE.CylinderGeometry( 0.25, 0.15, 0.8 );
links_geom["tail_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["tail_upper"] = new THREE.CylinderGeometry( 0.25, 0.2, 0.6 );
links_geom["tail_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["tail_spike"] = new THREE.IcosahedronGeometry( 0.45 );
links_geom["tail_spike"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw1_upper"] = new THREE.CylinderGeometry( 0.1, 0.1, 0.6 );
links_geom["claw1_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw1_middle"] = new THREE.CylinderGeometry( 0.15, 0.1, 0.5 );
links_geom["claw1_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw1_lower"] = new THREE.CylinderGeometry( 0.25, 0.1, 0.4 );
links_geom["claw1_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw1_pincer1"] = new THREE.ConeGeometry( 0.1, 0.3, 12);
links_geom["claw1_pincer1"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw1_pincer2"] = new THREE.ConeGeometry( 0.15, 0.4, 8);
links_geom["claw1_pincer2"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw2_upper"] = new THREE.CylinderGeometry( 0.1, 0.1, 0.6 );
links_geom["claw2_upper"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw2_middle"] = new THREE.CylinderGeometry( 0.15, 0.1, 0.5 );
links_geom["claw2_middle"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw2_lower"] = new THREE.CylinderGeometry( 0.25, 0.1, 0.4 );
links_geom["claw2_lower"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw2_pincer1"] = new THREE.ConeGeometry( 0.1, 0.3, 12);
links_geom["claw2_pincer1"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

links_geom["claw2_pincer2"] = new THREE.ConeGeometry( 0.14, 0.5, 8);
links_geom["claw2_pincer2"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0) );

