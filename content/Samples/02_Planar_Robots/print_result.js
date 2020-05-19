class includer{
    constructor(files){
        this.__not_already_included = true;
        this.__files =files;
    }

    include(cllbck){
        if(this.__not_already_included){
            let In = new Promise((res)=>{ res();});
            let this_ref = this;
            for(let i=0; i<this.__files.length; i++) {
                In = In.then(()=>{
                    return new Promise((res)=>{
                        let temp = this_ref.__include(this_ref.__files[i]);
                        temp.onload = ()=>{res();};
                    });
                });
            }
            In = In.then(cllbck);
            this.__not_already_included = false;
        }
        else cllbck();
    }
    __include(file){
        let temp = document.createElement("script");
        temp.setAttribute("src", file);
        document.body.appendChild(temp);
        return temp;
    }
}
let Includer =  new includer(["../../../src_JS/src/SVG_kit.js","Robot.js"]);

function print(wrapper, Problem, Result, Strategy, cllbck){

    Includer.include(()=>{

        function compute_BB_obstacles(){
            let BB = [0,0,0,0];
            for(let k=0; k<Problem.obstacles.length; k++){
                if(BB[0] > (Problem.obstacles[k][0] - Problem.obstacles[k][2])) BB[0] = Problem.obstacles[k][0] - Problem.obstacles[k][2];
                if(BB[1] < (Problem.obstacles[k][0] + Problem.obstacles[k][2])) BB[1] = Problem.obstacles[k][0] + Problem.obstacles[k][2];
                if(BB[2] > (Problem.obstacles[k][1] - Problem.obstacles[k][2])) BB[2] = Problem.obstacles[k][1] - Problem.obstacles[k][2];
                if(BB[3] < (Problem.obstacles[k][1] + Problem.obstacles[k][2])) BB[3] = Problem.obstacles[k][1] + Problem.obstacles[k][2];
            }
            return BB;
        }

        function compute_BB_robot(Robot){
            let BB = [0,0,0,0];
            for(let d=0;d<Robot.__links.length; d++){
                if(Robot.__links[d].p_cartesian[0] < BB[0]) BB[0] = Robot.__links[d].p_cartesian[0];
                if(Robot.__links[d].p_cartesian[0] > BB[1]) BB[1] = Robot.__links[d].p_cartesian[0];
                if(Robot.__links[d].p_cartesian[1] < BB[2]) BB[2] = Robot.__links[d].p_cartesian[1];
                if(Robot.__links[d].p_cartesian[1] > BB[3]) BB[3] = Robot.__links[d].p_cartesian[1];
            }
            if(Robot.__base[0] < BB[0]) BB[0] = Robot.__base[0];
            if(Robot.__base[0] > BB[1]) BB[1] = Robot.__base[0];
            if(Robot.__base[1] < BB[2]) BB[2] = Robot.__base[1];
            if(Robot.__base[1] > BB[3]) BB[3] = Robot.__base[1];
            return BB;
        }
        function compute_BB_robots(Robots){
            let BB = compute_BB_robot(Robots[0]);
            for(let k=1; k<Robots.length; k++) {
                let BB2 = compute_BB_robot(Robots[k]);
                BB = compute_BB(BB, BB2);
            }
            return BB;
        }

        function compute_BB(BB1, BB2){
            let BB = [];
            if(BB1[0] < BB2[0]) BB.push(BB1[0]);
            else                BB.push(BB2[0]);
            
            if(BB1[1] > BB2[1]) BB.push(BB1[1]);
            else                BB.push(BB2[1]);
            
            if(BB1[2] < BB2[2]) BB.push(BB1[2]);
            else                BB.push(BB2[2]);
            
            if(BB1[3] > BB2[3]) BB.push(BB1[3]);
            else                BB.push(BB2[3]);
    
            return BB;
        }
    
        function import_robot(color, opacity){
            let Robots = [];
            for(let r=0; r<Problem.robots.length; r++){
                let L = [];
                let dof = (Problem.robots[r].length-2)/2.0;
                for(let d=0; d<dof; d++) L.push(Problem.robots[r][2+d]);
                let rob = new Planar_Robot(canvas, L, [Problem.robots[r][0], Problem.robots[r][1]], color, "yellow");
                SVG_element.set_all("opacity" , opacity, rob.__get_SVG_links());
                SVG_element.set_all("opacity" , opacity, rob.__get_SVG_joint());
                Robots.push(rob);
            }
            return Robots;
        }

        function import_Q(Q){
            let Q2 = [];
            let c = 0;
            for(let r=0; r<Problem.robots.length; r++){
                let q=[];
                let dof = (Problem.robots[r].length-2)/2.0;
                for(let d=0; d<dof; d++){
                    q.push(Q[c] * 180.0/ 3.14159);
                    c++;
                }
                Q2.push(q);
            }
            return Q2;
        }

        function set_pose(Robots, Qs){
            for(let r=0; r<Problem.robots.length; r++) Robots[r].set_pose(Qs[r]);
        }

        function Interp(Q){
            function get_delta(Q_prev, Q_next){
                let Delta=[] , N=1;

                let k, kk;
                let N_att;
                for(k=0; k<Q_prev.length; k++){
                    for(kk=0; kk<Q_prev[k].length; kk++) {
                        N_att = Math.ceil(Math.abs(Q_next[k][kk] - Q_prev[k][kk]) / 4.0);
                        if(N_att > N) N = N_att;
                    }
                }
                for(k=0; k<Q_prev.length; k++){
                    let delta = [];
                    for(kk=0; kk<Q_prev[k].length; kk++) {
                        delta.push((Q_next[k][kk] - Q_prev[k][kk]) / (N*1.0));
                    }
                    Delta.push(delta);
                }

                return {Delta:Delta, N:N};
            }

            function get_summed(Q_prev, Q_delta){
                let kk;
                let Res= [];
                for(let k=0; k<Q_prev.length; k++){
                    let r =[];
                    for(kk=0; kk<Q_prev[k].length; kk++) r.push(Q_prev[k][kk] + Q_delta[k][kk]);
                    Res.push(r);
                }
                return Res;
            }

            Q2 = [Q[0]];
            let c = 0;
            for(let k=1; k<Q.length; k++){
                let info = get_delta(Q[k-1] , Q[k]);
                for(let n=0;n<info.N;n++){
                    Q2.push(get_summed(Q2[c] , info.Delta));
                    c++;
                }
            }
            return Q2;
        }

        let canvas = SVG_canvas.wrap_SVG_canvas(wrapper);    
        let BB = [0,0,0,0];

    //plot start end
        let Start = import_robot("aqua", 0.5);
        set_pose(Start, import_Q(Result.Start));
        let End = import_robot("#00ff2a", 0.5);
        set_pose(End, import_Q(Result.Target));
    //plot obstacles
        for(let k=0; k<Problem.obstacles.length; k++){
            let r = new SVG_circle( Problem.obstacles[k][2] , [Problem.obstacles[k][0], Problem.obstacles[k][1]] , canvas);
            r.fill("#ff00e1");
            r.set_stroke("yellow" , "0.1%");
        }
    //plot solution
        let Anim_Pr = null;
        let Anim_Qp = 0;
        let Anim_Q= [];
        if(Result[Strategy].Solution.length > 0){
            let Anim  = import_robot("aqua", 1.0);
            for(let k=0; k<Result[Strategy].Solution.length; k++){
                temp_Q = import_Q(Result[Strategy].Solution[k]);
                set_pose(Anim , temp_Q);
                BB = compute_BB(BB, compute_BB_robots(Anim));
                Anim_Q.push(temp_Q);
            }
            Anim_Q = Interp(Anim_Q);
        //add animation
            Anim_Pr = new Promise((res)=>{ res(); });
            function wayp_following(){
                Anim_Pr = Anim_Pr.then(()=>{
                    set_pose(Anim, Anim_Q[Anim_Qp]);
                    Anim_Qp++;
                    if(Anim_Qp == Anim_Q.length) Anim_Qp = 0;
                    setTimeout(()=>{
                        wayp_following();
                        res(); 
                    } , 100);
                });
            }
            wayp_following();
        }
        
        BB = compute_BB(BB, compute_BB_obstacles());
        BB = compute_BB(BB, compute_BB_robots(Start));
        BB = compute_BB(BB, compute_BB_robots(End));
        canvas.__SVG_frame.setAttributeNS(null, "viewBox", BB[0] + " " + BB[2] + " " + (BB[1] - BB[0])  + " " + (BB[3] - BB[2]));

        cllbck();
    });

}