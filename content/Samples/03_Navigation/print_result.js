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
let Includer =  new includer(["../../../src_JS/src/SVG_kit.js"]);

function print(wrapper, Problem, Result, Strategy, cllbck){

    Includer.include(()=>{

        function compute_BB_obstacles(){
            let BB = [0,0,0,0];
            for(let k=0; k<Problem.obstacles.length; k++){
                if(BB[0] > (Problem.obstacles[k][0] - Problem.obstacles[k][3])) BB[0] = Problem.obstacles[k][0] - Problem.obstacles[k][3];
                if(BB[1] < (Problem.obstacles[k][0] + Problem.obstacles[k][3])) BB[1] = Problem.obstacles[k][0] + Problem.obstacles[k][3];
                if(BB[2] > (Problem.obstacles[k][1] - Problem.obstacles[k][3])) BB[2] = Problem.obstacles[k][1] - Problem.obstacles[k][3];
                if(BB[3] < (Problem.obstacles[k][1] + Problem.obstacles[k][3])) BB[3] = Problem.obstacles[k][1] + Problem.obstacles[k][3];
            }
            return BB;
        }
    
        function compute_BB_solution(){
            let BB = [0,0,0,0];
            for(let t=0; t<Result[Strategy].Solution.length; t++){
                if(Result[Strategy].Solution[t][0] < BB[0]) BB[0] = Result[Strategy].Solution[t][0];
                if(Result[Strategy].Solution[t][0] > BB[1]) BB[1] = Result[Strategy].Solution[t][0];
                if(Result[Strategy].Solution[t][1] < BB[2]) BB[2] = Result[Strategy].Solution[t][1];
                if(Result[Strategy].Solution[t][1] > BB[3]) BB[3] = Result[Strategy].Solution[t][1];
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
    
        function print_conf(conf, sizes, color){
            let vehicle = SVG_rect.get_centered([0,0] , sizes[1], sizes[0], canvas);
            vehicle.fill(color);
            vehicle.set_rot_pos(conf[2]*180.0 / 3.141, [conf[0] ,conf[1] ]);
        }
    
        let BB = compute_BB( compute_BB_obstacles(), compute_BB_solution() );
    
        let canvas = SVG_canvas.wrap_SVG_canvas(wrapper);
        canvas.__SVG_frame.setAttributeNS(null, "viewBox", BB[0] + " " + BB[2] + " " + (BB[1] - BB[0])  + " " + (BB[3] - BB[2]));
    
        
    //plot obstacles
        for(let k=0; k<Problem.obstacles.length; k++){
            let r = new SVG_circle( Problem.obstacles[k][2] , [Problem.obstacles[k][0], Problem.obstacles[k][1]] , canvas);
            r.fill("#ff00e1");
            r.set_stroke("yellow" , "0.1%");
        }
    //plot solution
        if(Result[Strategy].Solution.length > 0){
            let s = new SVG_path( SVG_path.get_linear_segments(Result[Strategy].Solution), canvas);
            s.set_stroke("aqua", "0.5%");

            let vehicle = SVG_rect.get_centered([0,0] , Problem.info[6], Problem.info[5], canvas);
            vehicle.fill("yellow");
    
            let Prm = new Promise((res)=>{res(0);});
            function repeat_loop(){
                for(let p=0; p<Result[Strategy].Solution.length; p++){
                    Prm = Prm.then((p)=>{
                        return new Promise((res)=>{
                            setTimeout(()=>{
                                vehicle.set_rot_pos(Result[Strategy].Solution[p][2]*180.0 / 3.141, [Result[Strategy].Solution[p][0] ,Result[Strategy].Solution[p][1] ]);
                                res(p+1);
                            },10);
                        });
                    });    
                }
                Prm = Prm.then(()=>{ return new Promise((res)=>{ 
                    res(0); 
                    repeat_loop();
                }); });
            }
            repeat_loop();
        }
    //plot start end
        print_conf(Result['Start'], [ Problem.info[5], Problem.info[6]], "#5eff00");
        print_conf(Result['Target'], [ Problem.info[5], Problem.info[6]], "#ff0090");
    

        cllbck();
    });

}
