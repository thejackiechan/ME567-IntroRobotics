//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}

function matrix_multiply(m1,m2) { // assuming m1 * m2

    var mat = [];
    var v1 = [];
    var v2 = [];
    var i,j;

    if(m2[0].length === 1){ // implies matrix * column vector
        for(i = 0;i < m1.length;i++){
            v1 = m1[i];
            mat[i] = vector_multiply(v1,m2);
        }
        mat = matrix_transpose(mat);
    }else{
        for (i = 0;i < m1.length;i++){
            mat[i] = [];
            for(j = 0;j < m2[0].length;j++){
                v1 = m1[i];
                v2 = matrix_transpose(m2)[j];
                mat[i][j] = vector_multiply(v1,v2);
            }
        }
    }
    return mat;
}

function vector_sum(v1,v2,add_or_subtract){ // assumes v1 and v2 are of the form v = [a,b,c]

    var v = [];
    var i;

    for(i = 0;i < v1.length;i++){
        if(add_or_subtract == true){
            v[i] = v1[i] + v2[i];
        }else{
            v[i] = v1[i] - v2[i];
        }
    }
    return v;
}

function vector_multiply(v1,v2) { // element by element multiplication

    var value = 0;
    var i;

    for(i = 0;i < v1.length;i++) {
        value += v1[i]*v2[i];
    }
    return value;
}

function matrix_transpose(m1) {

    var mat = [];
    var i,j;

    if(typeof m1[0].length == 'undefined'){ // if m1 is a row vector 
        for (i = 0;i<m1.length;i++) { // CHECK THIS
            mat.push([m1[i]]); // check if works
        }
    }else{
        for (i=0;i<m1[0].length;i++) { // for each column of m2
            mat[i] = [];
            for (j=0;j<m1.length;j++) { // for each row of m2
                mat[i][j] = m1[j][i];
            }
        }
        if(mat[0].constructor === Array && mat[0].length > 1 && mat.length === 1){ // for transposing a column vec back to row vec
        mat = mat[0];
        }   
    }
    return mat;  
} 

function generate_identity(dimension) {

    var mat = [];
    var i,j;

    for(i = 0;i < dimension;i++){
        mat[i] = [];
        for (j=0;j<dimension;j++){
            if(i == j){
                mat[i][j] = 1;
            }else{
                mat[i][j] = 0;
            }
        }
    }
    return mat;
}

function vector_normalize(v){

    var magnitude = 0;
    var magnitude_squared = 0;
    var norm_vec = [];
    var i,j;

    for(i = 0;i < v.length;i++){
        magnitude_squared += Math.pow(v[i],2);
    }
    magnitude = Math.pow(magnitude_squared,0.5);

    for(j = 0;j < v.length;j++){
        norm_vec[j] = v[j]/magnitude;
    }
    return norm_vec;
}

function vector_cross(v1,v2){ 

    var cross = [(v1[1]*v2[2] - v1[2]*v2[1]),(v1[2]*v2[0] - v1[0]*v2[2]),(v1[0]*v2[1] - v1[1]*v2[0])]; 
    cross = matrix_transpose(cross);
    return cross; // returns column vec
}

function generate_translation_matrix(x,y,z){ // check if this works

    var mat = generate_identity(4);
    
    mat[0][mat[0].length - 1] = x;
    mat[1][mat[0].length - 1] = y;
    mat[2][mat[0].length - 1] = z;

    return mat;
}

function generate_rotation_matrix_X(theta){

    var mat = generate_identity(4);

    mat[1][1] = Math.cos(theta);
    mat[1][2] = -Math.sin(theta);
    mat[2][1] = Math.sin(theta);
    mat[2][2] = Math.cos(theta);

    return mat;
}

function generate_rotation_matrix_Y(theta){

    var mat = generate_identity(4);

    mat[0][0] = Math.cos(theta);
    mat[0][2] = Math.sin(theta);
    mat[2][0] = -Math.sin(theta);
    mat[2][2] = Math.cos(theta);

    return mat;
}

function generate_rotation_matrix_Z(theta){

    var mat = generate_identity(4);

    mat[0][0] = Math.cos(theta);
    mat[0][1] = -Math.sin(theta);
    mat[1][0] = Math.sin(theta);
    mat[1][1] = Math.cos(theta);

    return mat;
}

 function matrix_invert_affine(m1){ // assumes inputs are a square matrix 4x4 
   
    var R = matrix_copy(m1); // to ensure JS doesn't do anything funky with m1
    var last_row = R.pop(); // 3x4
    R = matrix_transpose(R); // 4x3
    var d = R.pop(); // extract translational elements as row vec
    var neg_d = scaleByAngle(-1,d); 
    neg_d = matrix_transpose(neg_d); // column vec used for matrix-vector multiplication
    var last_column = matrix_multiply(R,neg_d);
    var last_column = matrix_transpose(last_column); // row vec to push

    R = matrix_transpose(R); // turn R' to R temporarily
    R.push(last_column);
    R = matrix_transpose(R); // R is now a 3x4 with the correct rotational and translational elements
    R.push(last_row); // complete 4x4 inverse of m1

    return R;
 }

 function matrix_pseudoinverse(matrix){ // takes in a matrix A with dimensions NxM that is not invertible

    var rows = matrix.length;
    var cols = matrix[0].length;
    var mat_trans = matrix_transpose(matrix);

    if(rows > cols){ // left pseudoinverse

        var square = matrix_multiply(mat_trans,matrix);
        var inv = matrix_inverse(square);
        var mat = matrix_multiply(inv,mat_trans); 

    }else if(cols > rows){ // right pseudoinverse

        var square = matrix_multiply(matrix,mat_trans);
        var inv = matrix_inverse(square);
        var mat = matrix_multiply(mat_trans,inv);
    }
    return mat;
 }

function scaleByAngle(angle,vector){ // accepts both row or columns

    var i;

    for(i = 0;i < vector.length;i++){
        vector[i] = vector[i] * angle;
    }
    return vector; // returns row
}

    // STENCIL: reference matrix code has the following functions:
    //   matrix_multiply 
    //   matrix_transpose
    //   matrix_pseudoinverse // need left (more rows than columns) and right (more columns than rows)
    //   matrix_invert_affine
    //   vector_normalize
    //   vector_cross
    //   generate_identity
    //   generate_translation_matrix
    //   generate_rotation_matrix_X
    //   generate_rotation_matrix_Y
    //   generate_rotation_matrix_Z


// LU Decomposition 

// Implemented LU decomposition (with pivoting) routines for matrix inversion (matrix_inverse) and solving linear systems (linear_solve). 

function matrix_inverse(m1){ // assumes m1 is a 4x4

    var L,U,P;

    [L,U,P] = LU_decomposition(m1);
    return LU_inverse(L,U,P);
}

function linear_solve(m1,b){ // of the form Ax = b, m1 = A

    var m1_inv = matrix_inverse(m1);
    return matrix_multiply(m1_inv,b); // x = A^-1 * b
}

// Helper functions for LU Decomposition 

function pivot(m1){

    var i,j;
    var mat = generate_identity(m1.length); // assumes square --> # rows = # cols

    for(i = 0;i < mat.length;i++){
        var threshold = 0; 
        for(j = 0;j < mat[0].length;j++){  
            if(Math.abs(m1[i][j] > threshold)){
                threshold = Math.abs(m1[i][j]);
                var row_idx = i;
            }
        }
        if(i != row_idx){ // swap rows 
            var temp = mat[row_idx];
            mat[row_idx] = mat[i];
            mat[i] = temp;
        }
    }
    return mat;
}

function LU_decomposition(m1){ // algorithm adapted from https://www.researchgate.net/figure/LU-decomposition-algorithm-is-presented-in-a-The-resulting-matrices-of-the_fig2_220094411

    var i,j,k;

    var L = generate_identity(m1.length);
    var U = generate_identity(m1.length);
    var P = pivot(m1);
    var mat = matrix_multiply(P,m1);

    for(j = 0;j < m1[0].length;j++){ 
        for(i = 0;i < j + 1;i++){
            var upper_sum = 0;
            for(k = 0;k < i;k++){
                upper_sum += U[k][j]*L[i][k];
            } 
            U[i][j] = mat[i][j] - upper_sum;
        }
        for(i = j;i < mat.length;i++){
            var lower_sum = 0;
            for(k = 0;k < j;k++){
                lower_sum += U[k][j]*L[i][k];
            }
            L[i][j] = (mat[i][j] - lower_sum)/U[j][j];
        }
    }
    return [L, U, P]; // seems to work
}

function LU_inverse(L,U,P){ // assumes L is a 4x4, otherwise would need to do this recursively

    var L_inv = invert_L(L); 
    var U_inv = invert_U(U);

    var mat1 = matrix_multiply(L_inv,P);
    var mat2 = matrix_multiply(U_inv,mat1); // multiply U_inv * L_inv * P

    return mat2;
}

function invert_L(mat){ // in place modification to save memory

    for(var j = 0;j < mat.length;j++){
        mat[j][j] = 1/mat[j][j];
        for(var i = j + 1;i < mat.length;i++){
            var row = extract_elements(mat,i,i,j,i-1);
            var row = row[0];
            var column = extract_elements(mat,j,i-1,j,j);
            var product = vector_multiply(row,column);
            mat[i][j] = -product/mat[i][i]; // dividing by mat[i][i] is key
        }
    }
    return mat;
}

function invert_U(mat){ // UU^-1 = I --> (UU^-1)^T = I --> (U^-1)^T*U^T = I --> (U^-1)^T = (U^T)^-1, where U^T is lower triangular :)

    var mat = matrix_transpose(mat); // transpose U to make lower triangular
    mat = invert_L(mat); 
    return matrix_transpose(mat); // transpose back to get upper triangular
}

function extract_elements(m1,r1,r2,c1,c2){ // takes row indices r1:r2 and column indices c1:c2

    var mat = [];

    for(var i = r1;i <= r2;i++){
        mat.push([]);
        for(var j = c1;j <= c2;j++){
            mat[i - r1].push(m1[i][j]);
        }
    }
    return mat;
}

