
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }
    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }
    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}


    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    // initialize trees 
    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);

    // choose algorithm
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED), 2: rrt_star

    // initialize cost and bias for RRT* 
    if(rrt_alg == 2) T_a.vertices[0].vertex.cost = 0;
    toBias = false; // when set to true, RRT* will bias z_rand for quicker convergence

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;
    search_max_iterations = 100000; 

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}

// modify Init function to INITIALIZE RRT trees and other necessary variables

function robot_rrt_planner_iterate() {

    if (rrt_iterate && (Date.now()-cur_time > 1)) { // 10
        cur_time = Date.now();

        var eps = 0.8; // steplength

        var boundaries = createBounds(robot_boundary,q_goal_config,rrt_iter_count,eps,rrt_alg);

        if(rrt_alg == 0){ // basic rrt

            var q_rand = randomConfig(boundaries,q_start_config);
            var extend_outcome = extendRRT(T_a,q_rand,eps);
            if(extend_outcome[0] == "advanced"){
                var q_new = extend_outcome[1];

                var check = true;
                for(var i = 0;i < q_new.length;i++){
                    if(Math.abs(q_goal_config[i] - q_new[i]) > eps) check = false;
                }
                if(check){  // seems to never converge 
                    findPath(T_a);
                    rrt_iterate = false;
                    return "reached";
                }
            } 
        }else if(rrt_alg == 1){ // rrt connect

            var q_rand = randomConfig(boundaries,q_start_config);
            var extend_outcome = extendRRT(T_a,q_rand,eps);  

            if(extend_outcome[0] == "reached"){
                findPath(T_a);
                findPath(T_b);
                rrt_iterate = false;
                return "reached";
            }

            if(extend_outcome[0] != "trapped"){
                var q_target = extend_outcome[1]; 
                var connect_outcome = connectRRT(T_b,q_target,eps);
                if(connect_outcome[0] == "reached"){
                    findPath(T_a);
                    findPath(T_b);
                    rrt_iterate = false;
                    return "reached";
                }
            }
            [T_a,T_b] = swap(T_a,T_b);

        }else if(rrt_alg == 2){ // RRT*

            var z_min_idx,c_min,z_near_index; 
            var reach_distance = 2*eps;

            var z_rand = randomConfigBias(boundaries,q_start_config,q_goal_config,toBias,eps);
            var z_nearest_idx = findNearestNeighbor(z_rand,T_a);
            var z_nearest = T_a.vertices[z_nearest_idx].vertex; 
            var z_new = steer(z_nearest,z_rand,eps);
            var config_in_collision = kineval.poseIsCollision(z_new);

            if(!config_in_collision){
                var Z_near_idx = near(T_a,z_new,reach_distance); 
                [z_min_idx,c_min] = chooseParent(T_a,Z_near_idx,z_nearest_idx,z_new,eps);
                T_a = insertNode(T_a,z_new,z_min_idx,c_min); 
                T_a = reWire(T_a,Z_near_idx,z_new,z_min_idx,eps); 
            }

            // If points are sampled near the goal, let's bias random points to accelerate convergence process
            // /* comment section for no bias 
            var check = checkBias(q_goal_config,z_new,eps);
            if(check) toBias = true;
            // */

            check = finalCheck(q_goal_config,z_new,eps);

            if(check){
                findPath(T_a);
                addGoal(q_goal_config); // fencepost
                rrt_iterate = false;
                return "reached";
            }
        }

        if(rrt_iter_count > search_max_iterations){
            rrt_iterate = false;
            return "failed";
        }else{
            rrt_iter_count++;
            return "extended";
        }
    }
}

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations
    // Modify to perform single RRT-Connect iteration based on current RRT trees

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {

    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}

function removeTreeEdge(tree,q1_idx,q2_idx) {
    // remove 1st edge
    var i;

    for (i = 0;i < tree.vertices[q1_idx].edges.length;i++){
        if(tree.vertices[q1_idx].edges[i] == q2_idx) 
            tree.vertices[q1_idx].edges.splice(i,1);
    }
    // remove 2nd edge
    for (i = 0; i < tree.vertices[q2_idx].edges.length;i++){
        if(tree.vertices[q2_idx].edges[i] == q1_idx) 
            tree.vertices[q2_idx].edges.splice(i,1);
    }
}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////


    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

function randomConfig(ranges,q){

    var q_rand = q.slice(); // creates a copy of q
    var range_x = ranges[0][1] - ranges[0][0]; // diff in x
    var range_z = ranges[1][1] - ranges[1][0]; // diff in y
    var range_cont = [-2*Math.PI,2*Math.PI];
    var range_angles = range_cont[1] - range_cont[0]; // 4PI diff


    for(var i = 0;i < q_rand.length;i++){
        if(i == 0){ // x pos
            q_rand[i] = ranges[0][0] + Math.random()*range_x;
        }else if(i == 2){ // z pos
            q_rand[i] = ranges[1][0] + Math.random()*range_z;
        }else if(i == 4){
            //q_rand[i] = 0;
            q_rand[i] = range_cont[0] + Math.random()*range_angles; 
        }else if(i > 5){ // after rpy
            var j = robot.joints[q_index[i]];
            if(j.type == "continuous" || typeof j.type == "undefined"){
                q_rand[i] = range_cont[0] + Math.random()*range_angles;
            }else if(j.type == "prismatic" || j.type == "revolute"){
                q_rand[i] = j.limit.lower + Math.random()*(j.limit.upper - j.limit.lower);
            } // don't touch fixed joints b/c its the same
        }
    }
    return q_rand;
}

function findNearestNeighbor(q,tree){

    var min_d = Math.pow(10,1000); 

    for(var i = 0;i < tree.vertices.length;i++){
        var d = Euclid_distance(tree.vertices[i].vertex,q);
        if(d < min_d){
            min_d = d;
            var d_index = i;
        } 
    }
    return d_index; // returns tree configuration that is nearest 
}

function Euclid_distance(q1,q2){ // q1 and q2 must be the same size

    var sum_squares = 0;
    for(var i = 0;i < q1.length;i++){
        sum_squares += Math.pow(q1[i] - q2[i], 2);
    }
    return Math.sqrt(sum_squares);
}

function newConfig(q_rand,q_near,step_length){ // q = q_rand = q_target

    var q_new = [];
    var norm = Euclid_distance(q_rand,q_near); // calcs magnitude

    for(var i = 0;i < q_rand.length;i++){
        q_new[i] = step_length * (q_rand[i] - q_near[i])/norm + q_near[i];
    }
    return q_new; 
}

function extendRRT(tree,q_rand,step_length){ 
    var q_near_idx = findNearestNeighbor(q_rand,tree); // edit this
    var q_near = tree.vertices[q_near_idx].vertex;
    var q_new = newConfig(q_rand,q_near,step_length); 
    var config_in_collision = kineval.poseIsCollision(q_new);

    if(!config_in_collision){ // if config is a valid config (i.e. not in collision)
        tree_add_vertex(tree,q_new);
        tree_add_edge(tree,q_near_idx,tree.newest);

        var check_reached = true;
        for(var i = 0;i < q_new.length;i++){
            if(Math.abs(q_rand[i] - q_new[i]) > step_length) check_reached = false;
        }
        if(check_reached){ // no longer reaching goal but q_target passed in
            var state = "reached";
        }else{
            var state = "advanced";
        }
    }else{
        var state = "trapped";  
    }
    return [state,q_new];
}

function findPath(tree){

    var flag = "goal";
    var path = {};
    path.vertices = [];

    path.vertices[0] = {};
    var vertice = tree.vertices[tree.newest];
    path.vertices[0].vertex = vertice.vertex;

     while(Euclid_distance(vertice.vertex,q_start_config) != 0 && Euclid_distance(vertice.vertex,q_goal_config) != 0){ 
        vertice = vertice.edges[0];
        var new_vertice = {};
        new_vertice.vertex = vertice.vertex;
        path.vertices.push(new_vertice);

        if(Euclid_distance(vertice.vertex,q_start_config) == 0) flag = "start";
     }

     for(var i = 0;i < path.vertices.length;i++){
        add_config_origin_indicator_geom(path.vertices[i]);
        path.vertices[i].geom.material.color = {r:1,g:0,b:0};
        if(flag == "start"){
            kineval.motion_plan.unshift(path.vertices[i]);
        }else{
            kineval.motion_plan.push(path.vertices[i]); 
        }
     }
 }

 function connectRRT(tree,q_target,step_length){ // need extend to give back q_new if advanced or 
    while(true){
        var S = extendRRT(tree,q_target,step_length); // need to check if q_new is near goal
        if(S[0] != "advanced") break;
    }
    return S;
}

function swap(tree_a,tree_b){
    var temp = tree_a;  // swaps pointers so I need to return
    tree_a = tree_b;
    tree_b = temp;
    return [tree_a,tree_b];
}

function createBounds(bounds,q_goal,iter_count,step_length,algorithm){

    var boundaries = [[bounds[0][0] - 2*step_length,bounds[1][0] + 2*step_length],[bounds[0][2] - 2*step_length,bounds[1][2] + 2*step_length]];
    var center = [(bounds[0][0] + bounds[1][0])/2,(bounds[0][2] + bounds[1][2])/2];

    if(iter_count > 400 && iter_count % 3 == 0 && algorithm == 1){ //begin sampling near the center to help robot break out of any "dead ends"
        boundaries = [[center[0] - step_length, center[0] + step_length],[center[1] - step_length,center[1] + step_length]];
    }else if(iter_count > 1000 && iter_count % 10 == 0 && algorithm == 2){ // sample from middle
            boundaries = [[q_goal[0] - step_length, q_goal[0] + step_length],[q_goal[2] - step_length, q_goal[2] + step_length]];
    }  
    return boundaries;
}

//////////////////////////////////////////////////
/////     RRT-Star IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

function randomConfigBias(ranges,z_start,z_goal,bias,step_length){

    var i;

    if(!bias){ // this is basically the same as randomConfig
        var z_rand = z_start.slice(); // creates a copy of q
        var range_x = ranges[0][1] - ranges[0][0]; // diff in x
        var range_z = ranges[1][1] - ranges[1][0]; // diff in y
        var range_cont = [-2*Math.PI,2*Math.PI];
        var range_angles = range_cont[1] - range_cont[0]; // 4PI diff

        for(i = 0;i < z_rand.length;i++){

            if(i == 0){ // x pos
                z_rand[i] = ranges[0][0] + Math.random()*range_x;
            }else if(i == 2){ // z pos
                z_rand[i] = ranges[1][0] + Math.random()*range_z;
            }else if(i == 4){
                z_rand[i] = range_cont[0] + Math.random()*range_angles; 
            }else if(i > 5){ // after rpy
                var j = robot.joints[q_index[i]];
                if(j.type == "continuous" || typeof j.type == "undefined"){
                    z_rand[i] = range_cont[0] + Math.random()*range_angles;
                }else if(j.type == "prismatic" || j.type == "revolute"){
                    z_rand[i] = j.limit.lower + Math.random()*(j.limit.upper - j.limit.lower);
                } // don't touch fixed joints b/c its the same
            }
        }
    }else{ // this part helps RRT converge faster by biasing z_rand 
        var z_rand = z_goal.slice();
        var range = 2*step_length;

        for(i = 0;i < z_rand.length;i++){
            if(i == 0 || i == 2 || i == 4){ // x pos
                z_rand[i] = z_rand[i] - step_length + Math.random()*range;
            }else if(i > 5){ // after rpy
                var j = robot.joints[q_index[i]];
                if(j.type == "continuous" || typeof j.type == "undefined"){
                    z_rand[i] = z_rand[i] - step_length + Math.random()*range;
                }else if(j.type == "prismatic" || j.type == "revolute"){
                    var test = j.limit.lower + Math.random()*(j.limit.upper - j.limit.lower);
                    if(test > step_length){
                        z_rand[i] = z_rand[i] - step_length + Math.random()*range;
                    }
                } // fixed joints should correspond to 0 angle, q_goal_config happens to be all 0's
            }
        }
    }
    return z_rand;
}

function steer(z_nearest,z_rand,step_length){
    var z_new = newConfig(z_rand,z_nearest,step_length);
    return z_new;
}

function near(tree,z_new,reach_distance){ // returns array of indices of vertices in T_a that is within reach_distance from z_new

    var Z_near_idx = [];

    for(var i = 0;i < tree.vertices.length;i++){
        var distance = Euclid_distance(tree.vertices[i].vertex,z_new);
        if(distance <= reach_distance){
            Z_near_idx.push(i); // i is the index of vertice in tree
        }
    }
    return Z_near_idx;
}

function chooseParent(tree,Z_near_idx,z_nearest_idx,z_new,step_length){

    var z_min_idx = z_nearest_idx;
    var z_nearest = tree.vertices[z_nearest_idx].vertex;
    var c_min = z_nearest.cost + Euclid_distance(z_nearest,z_new); 

    for(var i = 0;i < Z_near_idx.length;i++){

        var z_near = tree.vertices[Z_near_idx[i]].vertex;
        var c_prime = z_near.cost + Euclid_distance(z_near,z_new); 
        if(c_prime < c_min){  // cost(z_new) = c_min
            z_min_idx = Z_near_idx[i]; // need index for drawing edges
            c_min = c_prime;
        }  
    }
    return [z_min_idx,c_min]; 
}

function insertNode(tree,z_new,parent_idx,cost){ 

    tree_add_vertex(tree,z_new);
    tree_add_edge(tree,parent_idx,tree.newest); 
    z_new.cost = cost; // assign cost to new vertex
    z_new.parent_idx = parent_idx; // set parent info 
    return tree;
}

function reWire(tree,Z_near_idx,z_new,z_min_idx,step_length){
    
    for(var i = 0;i < Z_near_idx.length;i++){
        if(Z_near_idx[i] != z_min_idx){
            var z_near = tree.vertices[Z_near_idx[i]].vertex;
            var dist = Euclid_distance(z_near,z_new);
            var x_prime = steer(z_new,z_near,step_length);

            if(!kineval.poseIsCollision(x_prime) && (z_new.cost + dist) < z_near.cost){
                tree = reconnect(z_new,Z_near_idx[i],tree);
            }
        }
    }
    return tree;
}

function reconnect(z_new,z_near_index,tree){

    removeTreeEdge(tree,tree.vertices[z_near_index].vertex.parent_idx,z_near_index); 
    var z_near = tree.vertices[z_near_index].vertex;
    z_near.parent_idx = tree.newest;
    tree_add_edge(tree,tree.newest,z_near_index); 
    return tree;
}

// Helper functions for RRT*

function checkBias(q_goal,z_new,step_length){ // begin biasing random points once xyz,rpy of z_new is close to goal xyz,rpy

    var joint_angle_idx = 6; // 0,1,2,3,4,5 indices of z_rand represent xyzrpy
    var check = true;
            
    for(var i = 0;i < joint_angle_idx;i++){
        var value = Math.abs(q_goal[i] - z_new[i]);
        if(value > step_length) check = false;
    }
    return check;
}

function finalCheck(q_goal,z_new,step_length){

    var check = true;
    for(var i = 0;i < z_new.length;i++){ // checks every component of z_new
        var value = Math.abs(q_goal[i] - z_new[i]);
        if(value > step_length) check = false; 
    }
    return check;
}

function addGoal(q_goal){

    var new_vertice = {};
    new_vertice.vertex = q_goal; // functionalize
    add_config_origin_indicator_geom(new_vertice);
    new_vertice.geom.material.color = {r:1,g:0,b:0};
    kineval.motion_plan.push(new_vertice);
}

//////////////////////////////////////////////////
/////     A* IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// need .priority, .queued, .distance fields, also need to maintain a visit_queue that gets heaped based on priority
// f_score heuristic defined as neighbor.distance + Euclidean distance from neighbor to goal
// Idea is to immplement A* for base planning and RRT for joint angles and orientation, whenever we generate a valid position, run RRT until we obtain a valid 
// configuration for the angles before pushing it into q. Should find shortest path. 

minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    heap.push(new_element);
    new_element_location = heap.length - 1;


    while(new_element_location != 0 && heap[new_element_location].priority < heap[Math.floor((new_element_location - 1)/2)].priority){ // no longer .priority
        parent_location = Math.floor((new_element_location - 1)/2); // cannot define this before while loop b/c parent_location can change  
        temp = heap[parent_location]; // temp = parent value
        heap[parent_location] = heap[new_element_location]; // parent = new element value
        heap[new_element_location] = temp; // replace new element with parent value
        new_element_location = parent_location;
    }
    return heap;
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;

// define extract function for min binary heap
function minheap_extract(heap) {

  if(heap.length > 1){
    swap_location = 0;
    root_element = heap[swap_location];
    last_element = heap.pop();
    heap[swap_location] = last_element;

    while((((2*swap_location) + 1) < heap.length && (heap[swap_location].priority > heap[(2*swap_location) + 1].priority))
        || (((2*swap_location) + 2) < heap.length && (heap[swap_location].priority > heap[(2*swap_location) + 2].priority))){

        child1_location = (2*swap_location) + 1; // cannot define these locations before while loop because swap_location can change
        child2_location = (2*swap_location) + 2;

        if(child2_location < heap.length){
            if(heap[child1_location].priority < heap[child2_location].priority){
                priority = child1_location;
            }else{
                priority = child2_location;
            }
        }else{
            priority = child1_location; // for when child2 is not located in heap
        }
        temp = heap[priority];
        heap[priority] = heap[swap_location];
        heap[swap_location] = temp;
        swap_location = priority;
    }
    }else if(heap.length == 1){
        root_element = heap.pop();
    }
    return root_element;
}

minheaper.extract = minheap_extract;
