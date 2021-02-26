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
let Includer =  new includer(["../../src_JS/src/SVG_kit.js"]);

function print(wrapper, Problem, Result, Strategy, cllbck){

    Includer.include(()=>{

        function compute_BB_obstacles(){
            let BB = [0,0,0,0];
            for(let k=0; k<Problem.length; k++){
				if(Problem[k][0] < BB[0]) BB[0] = Problem[k][0];
				if(Problem[k][2] > BB[1]) BB[1] = Problem[k][2];
				if(Problem[k][1] < BB[2]) BB[2] = Problem[k][1];
				if(Problem[k][3] > BB[3]) BB[3] = Problem[k][3];
            }
            return BB;
        }
    
        function compute_BB_trees(){
            let BB = [0,0,0,0];
            for(let t=0; t<Result[Strategy].Trees.length; t++){
                for(let tt=0; tt<Result[Strategy].Trees[t].Tree.length; tt++){
                    if(Result[Strategy].Trees[t].Tree[tt]['E'][0] < BB[0]) BB[0] = Result[Strategy].Trees[t].Tree[tt]['E'][0];
                    if(Result[Strategy].Trees[t].Tree[tt]['E'][0] > BB[1]) BB[1] = Result[Strategy].Trees[t].Tree[tt]['E'][0];
                    if(Result[Strategy].Trees[t].Tree[tt]['E'][1] < BB[2]) BB[2] = Result[Strategy].Trees[t].Tree[tt]['E'][1];
                    if(Result[Strategy].Trees[t].Tree[tt]['E'][1] > BB[3]) BB[3] = Result[Strategy].Trees[t].Tree[tt]['E'][1];
                }
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
    
        function print_tree(t, color, canvas){
            for(let tt=0; tt<Result[Strategy].Trees[t].Tree.length; tt++){
                let s = new SVG_path( SVG_path.get_linear_segments([Result[Strategy].Trees[t].Tree[tt]['E'],Result[Strategy].Trees[t].Tree[tt]['S']]), canvas);
                s.set_stroke(color, "0.15%");
            }
        }
    
        let BB = compute_BB( compute_BB_obstacles(), compute_BB_trees() );
    
        let canvas = SVG_canvas.wrap_SVG_canvas(wrapper);
        canvas.__SVG_frame.setAttributeNS(null, "viewBox", BB[0] + " " + BB[2] + " " + (BB[1] - BB[0])  + " " + (BB[3] - BB[2]));
    
     
    //plot start end
        let S = new SVG_circle("1%", Result['Start'], canvas);
        S.fill("aqua");
        let E = new SVG_circle("1%", Result['Target'], canvas);
        E.fill("#00ff2a");
    //plot obstacles
        for(let k=0; k<Problem.length; k++){
            let r = new SVG_rect( (Problem[k][2] - Problem[k][0]) , (Problem[k][3] - Problem[k][1]),canvas , Problem[k][0], Problem[k][1]);
            r.fill("#ff6600");
        }
    //plot trees 
        print_tree(0, "aqua", canvas);
        if(Result[Strategy].Trees.length > 1) print_tree(1, "#00ff2a", canvas);
    //plot solution
        if(Result[Strategy].Solution.length > 0){
            let s = new SVG_path( SVG_path.get_linear_segments(Result[Strategy].Solution), canvas);
            s.set_stroke("blue", "0.5%");
        }

        cllbck();
    });

}