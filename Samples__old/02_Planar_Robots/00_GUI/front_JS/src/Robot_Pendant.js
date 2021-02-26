class Pendant{
    constructor(Height, Width, Robot, color_background="white", color_front="black"){
    // data ////////////////////////
        this.div = null;
    ////////////////////////////////

        this.__create_handlers(Height, Width, Robot, color_background, color_front);
    }

    
    attach(wrapper){ wrapper.appendChild(this.div); };


    __create_handlers(Height, Width, Robot, color_background, color_front){
        this.div = document.createElement("div");
        this.div.style.height = Height + "px";
        this.div.style.width = Width + "px";

        let h = Height / (Robot.get_dof()*1.0);
        let K =Robot.get_dof();

        for(let k=0; k<K; k++){
            let hnd = new handle(Robot.__q_min[k], Robot.__q_max[k], h, Width-2*h, color_background, color_front);
            hnd.attach(this.div);            
            let hnd_fired = false;
            Robot.add_onchange((Q)=>{ 
                if(!hnd_fired) hnd.set_value(Q[k]); 
            });
            hnd.add_onchange((val)=>{ 
                hnd_fired = true;
                Robot.set_pose_single(val,k); 
                hnd_fired = false;
            });
        }
        Robot.set_pose(Robot.get_pose());
    }
}