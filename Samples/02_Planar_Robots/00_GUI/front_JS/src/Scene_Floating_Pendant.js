const H_head = 30;
const H_hndl = 30;
const W = 200;
const W_menu = 5;
const W_comm = 50;

class Pendant_floating {
    constructor(robot){
    // data ////////////////////////
        this.__robot = robot;
        this.__canvas = null;
        this.__H_pendant = null;
        this.__div_float = null;
        this.__div_float_achor = null;
        this.__div_comm1 = null; //always avilable commands
        this.__div_comm2 = null; //comands available only in case a target pose was previously selected

        this.__Q_target = null; ///
        this.__saved_poses = [];

        this.__show_hide_comm_panel = false;
        this.__show_poses = false;
    ////////////////////////////////

        this.__H_pendant = robot.get_dof()*H_hndl;
        this.__canvas = robot.__canvas;
        this.__init_floating();
        this.__init_anchor();
        this.__add_beh_show_hide_pendant();

        this.__init_comm1();
        this.__init_comm2();
        this.__add_beh_show_hide_comm_panel();
    };


    get_Q_trg(){ 
        if(this.__Q_target === null) return null;
        else return this.__Q_target.get_pose(); 
    };
    detach(){ 
        if(this.__show_poses){ 
            for(let k=0; k<this.__saved_poses.length;k++) detach_pose(this.__saved_poses[k]); 
            this.__show_poses = false;
        }
        try { this.__div_float_achor.detach(); } catch (error) {}
        try { this.__div_float.detach(); } catch (error) {}
    };


    __init_floating(){
        this.__div_float = new Floating_div(W , H_head + this.__H_pendant, H_head, "white", "#442d4f");
        this.__div_float.detach();
        this.__div_float.get_body().style.display = "flex";

        let handlers = new Pendant(this.__H_pendant, W-W_menu, this.__robot, "white", "#442d4f");
        handlers.attach(this.__div_float.get_body());  

        add_popup(this.__div_float.get_header(), "This is the pendant. Use the below cursors to set each robot joint position. Click the right border to show additional commands (e.s. save new poses and se the target). Click again the robot base to hide the pendant.", null);

        let menu = new SVG_canvas([0,0,10,10]);
        menu.__SVG_frame.setAttributeNS(null, "preserveAspectRatio", "none");
        this.__div_float.get_body().appendChild(menu.__SVG_frame);
        menu.__SVG_frame.setAttributeNS(null, "width" , W_menu + "px");
        menu.__SVG_frame.setAttributeNS(null, "height" , this.__H_pendant + "px");
        let menu_arrw = new SVG_path(SVG_path.get_linear_segments([[0.5,6],[9.5,5],[0.5,4]]), menu);
        menu_arrw.fill("white");
        opacity_mouseover(menu.__SVG_frame, 0.5);
    };
    __init_anchor(){
        let b = this.__robot.__base;
        let p0 = [0,0];
        this.__div_float_achor = new SVG_path(SVG_path.get_linear_segments([[b[0], b[1]],[p0[0],p0[1]]]) , this.__canvas);
        this.__div_float_achor.detach();
        this.__div_float_achor.set_stroke("yellow", "0.1%");
        this.__div_float_achor.set_dash();
        let this_ref = this;
        this.__div_float.set_onmove((p)=>{
            let p_cnv = this_ref.__canvas.pos_in_canvas(p);
            this.__div_float_achor.set("d", SVG_path.get_linear_segments([[b[0], b[1]],[p_cnv[0],p_cnv[1]]]));
        });
    }
    __add_beh_show_hide_pendant(){
        let this_ref = this;
        click_on_off_behaviour(this.__robot.__links[0].circle_SVG.__element , 
            ()=>{ this_ref.__div_float.attach(); this_ref.__div_float_achor.attach(this_ref.__canvas, 3); },
            ()=>{ this_ref.__div_float.detach(); this_ref.__div_float_achor.detach();} 
        );
    }

    __init_comm1(){
        this.__div_comm1 = document.createElement("div");
        this.__div_comm1.style.height = this.__H_pendant + "px";
        this.__div_comm1.style.width = W_comm + "px";
        this.__init_comm_show();
        this.__init_comm_add();
    }
    __init_comm2(){
        this.__div_comm2 = document.createElement("div");
        this.__div_comm2.style.height = this.__H_pendant + "px";
        this.__div_comm2.style.width = W_comm + "px";
        this.__init_comm_move();
        this.__init_comm_del();
    }
    __add_beh_show_hide_comm_panel(){
        let this_ref = this;
        click_on_off_behaviour(this.__div_float.get_body().childNodes[1] , 
            ()=>{ this_ref.__show_hide_comm_panel = true;  this_ref.__update_comm_panel(); },
            ()=>{ this_ref.__show_hide_comm_panel = false; this_ref.__update_comm_panel(); } 
        );
    }
    __update_comm_panel(){
        let this_ref = this;
        function set_W_flaot(w){ this_ref.__div_float.__div.style.width = w; }
        set_W_flaot(W);
        try { this.__div_float.get_body().removeChild(this.__div_comm1); } catch (error) {}
        try { this.__div_float.get_body().removeChild(this.__div_comm2); } catch (error) {}
        if(this.__show_hide_comm_panel){
            if(this.__Q_target === null){
                set_W_flaot(W + W_comm);
                this.__div_float.get_body().appendChild(this.__div_comm1);
            }
            else{
                set_W_flaot(W + 2 * W_comm);
                this.__div_float.get_body().appendChild(this.__div_comm1);
                this.__div_float.get_body().appendChild(this.__div_comm2);
            }
        }
    }
    __init_comm_show(){
        let temp = get_div_with_image(["100%", "50%"], "./image/show_poses.svg");
        this.__div_comm1.appendChild(temp);
        let this_ref = this;
        click_on_off_behaviour(temp , 
            ()=>{ this_ref.__show_poses = true; for(let k=0; k<this_ref.__saved_poses.length;k++) attach_pose(this_ref.__saved_poses[k]); },
            ()=>{ this_ref.__show_poses = false; for(let k=0; k<this_ref.__saved_poses.length;k++) detach_pose(this_ref.__saved_poses[k]); } 
        );
        add_popup(temp, "Show all the saved poses. ", 200);
    }
    __init_comm_add(){
        let temp = get_div_with_image(["100%", "50%"], "./image/add_pose.svg");
        this.__div_comm1.appendChild(temp);
        let this_ref = this;
        temp.onclick = ()=>{
            let new_pose = new Planar_Robot(this_ref.__canvas, this_ref.__robot.get_lengths(), this_ref.__robot.get_base(), 
            this_ref.__robot.__links[0].circle_SVG.get("fill"), this_ref.__robot.__links[0].circle_SVG.get("stroke"));
            new_pose.set_pose(this_ref.__robot.get_pose());
            new_pose.detach();

            SVG_element.set_all("opacity", 0.5, new_pose.__get_SVG_links());
            function set_as_target(el){ el.__element.onclick = ()=>{ this_ref.__set_Q_target(new_pose); } }
            let shapes = new_pose.__get_SVG_shapes();
            for(let k=0; k<shapes.length; k++) set_as_target(shapes[k]);

            this_ref.__saved_poses.push(new_pose);
            if(this_ref.__show_poses) attach_pose(new_pose);
        };
        add_popup(temp, "Add the current pose to the saved ones. Click a saved pose to make it the target (borders will be increased).", 200);
    }
    __init_comm_move(){
        let temp = get_div_with_image(["100%", "50%"], "./image/move_to_pose.svg");
        this.__div_comm2.appendChild(temp);
        let this_ref = this;
        temp.onclick = ()=>{ this_ref.__robot.set_pose(this_ref.__Q_target.get_pose()); }
        add_popup(temp, "Move the robot to the actual target pose ", 200);
    }
    __init_comm_del(){
        let temp = get_div_with_image(["100%", "50%"], "./image/remove_pose.svg");
        this.__div_comm2.appendChild( temp);
        let this_ref = this;
        temp.onclick = ()=>{
            this_ref.__Q_target.detach();
            remove(this_ref.__saved_poses , this_ref.__Q_target);
            this_ref.__Q_target = null;
            this_ref.__update_comm_panel();
        }
        add_popup(temp, "Delete the actual target pose", 200);
    }
    __set_Q_target(pose){
        let shapes = pose.__get_SVG_shapes();
        if(pose === this.__Q_target){
            SVG_element.set_all("stroke-width", "0.2%", shapes);
            this.__Q_target = null;
        }
        else{
            if(this.__Q_target !== null){
                let shapes2 = this.__Q_target.__get_SVG_shapes();
                SVG_element.set_all("stroke-width", "0.2%", shapes2);
            }
            SVG_element.set_all("stroke-width", "0.4%", shapes);
            this.__Q_target = pose;
        }
        this.__update_comm_panel();
    }
}




function get_div_with_image(sizes, relative_path){
    let temp = document.createElement("div");
    temp.style.width = sizes[0];
    temp.style.height = sizes[1];
    let img = SVG_image.get_div_image(relative_path);
    img.__SVG_frame.setAttributeNS(null, "preserveAspectRatio" , "xMidYMid meet");
    temp.appendChild(img.__SVG_frame);
    opacity_mouseover(temp, 0.5);
    set_bord(temp);
    return temp;
}

function attach_pose(robot){
    let canvas = robot.__canvas;
    SVG_element.attach_all(canvas ,robot.__get_SVG_links() , 1);
    let joint = robot.__get_SVG_joint();
    joint.shift();
    SVG_element.attach_all( canvas, joint);
}

function detach_pose(robot){
    let canvas = robot.__canvas;
    SVG_element.detach_all(robot.__get_SVG_links());
    let joint = robot.__get_SVG_joint();
    joint.shift();
    SVG_element.detach_all(joint);
}