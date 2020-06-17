function sum(a, b){
    let res = [];
    let S = a.length;
    for(let s = 0; s<S; s++){
        res.push(a[s] + b[s]);
    }
    return res;
}

function diff(a,b){
    let res = [];
    let S = a.length;
    for(let s = 0; s<S; s++){
        res.push(a[s] - b[s]);
    }
    return res;
}

function dot(a,b){
    let res = 0;
    let S = a.length;
    for(let s = 0; s<S; s++){
        res = res + a[s] * b[s];
    }
    return res;
}

function get_transpose(Mat){
    let res = [];
    let R=Mat.length, r, C = Mat[0].length, c;
    for(c = 0; c<C; c++){
        res.push([]);
        for(r = 0; r<R; r++){
            res[c].push(Mat[r][c]);
        }
    }
    return res;
}

function product_Mat_Vec(Mat, Vec){
    let R = Mat.length;
    let res = [];
    for(let r=0; r<R; r++){
        res.push(dot(Mat[r] , Vec));
    }
    return res;
}

function product_Mat_Mat(M1, M2){
    let R = M1.length, C = M2[0].length,r, r2,c, S = M1[0].length;
    let res = [];
    for(r=0; r<R; r++){
        res.push([]);
        for(c=0; c<C; c++){
            res[r].push(0);
            for(r2 = 0; r2 <S; r2++){
                res[r][c] = res[r][c] + M1[r][r2] * M2[r2][c];
            }
        }
    }
    return res;
}

function rad_2_grad(Q){
    let S= Q.length;
    for(let s=0;s<S; s++){
        Q[s] =  Q[s] *180.0 / 3.14159;
    }
}

function grad_2_rad(Q){
    let S= Q.length;
    for(let s=0;s<S; s++){
        Q[s] =  Q[s] * 3.14159 / 180.0;
    }
}

function get_psinvJ(J){
    let a = dot(J[0], J[0]);
    let b = dot(J[1], J[1]);
    let c = dot(J[0], J[1]);
    let delta = a*b - c*c;
    if(Math.abs(delta) < 1e-4) throw 0;
    delta =  1.0 / delta;
    let res = product_Mat_Mat(get_transpose(J), [[delta*b, -delta*c], [-delta*c, delta*a]]);
    return res;
}