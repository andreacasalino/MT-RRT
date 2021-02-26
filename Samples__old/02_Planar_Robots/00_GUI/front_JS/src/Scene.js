const use_conn  = true;
const let_anim = 0.1; //[s]
const use_debug_bubble = false;

class Scene{
    constructor(){
    // data ////////////////////////
        this.__div = null; //on top the command panel, on bottom the svg canvas
        this.__connection = null;
        this.__settings=null;
        this.__canvas = null;
        this.__obstacles = [];
        this.__robots = []; //{robot, pendant}
        this.__promise_robots_motion = null;
    ///////////////////////////////

        let this_ref = this;
        new Promise((res)=>{
            if(use_conn) this_ref.__connection = new Connection( 3001, res);
            else res();
        }).then(()=>{
            this_ref.__init_head_body();
            this_ref.__init_commands();
            this_ref.__init_settings();
            window.onbeforeunload = ()=>{ this_ref.__send_command("kill" , "null"); }
            this_ref.__prm_motion = new Promise((res)=>{ res(); });
        });
    }


    __init_head_body(){
        this.__div = document.createElement("div");
        document.body.appendChild(this.__div);
        this.__div.style.width = "100vw";
        this.__div.style.height = "100vh";
        
        let comm_panel = document.createElement("div");
        this.__div.appendChild(comm_panel);
        comm_panel.style.width = "100%";
        comm_panel.style.height = "8%";
        set_bord(comm_panel);
        comm_panel.style.backgroundColor = "#442d4f";
        comm_panel.style.display = "flex";

        let canvas = document.createElement("div");
        this.__div.appendChild(canvas);
        canvas.style.width = "100%";
        canvas.style.height = "92%";
        canvas.style.backgroundColor = "black";
        this.__canvas = SVG_canvas.wrap_SVG_canvas(canvas, 4);
    };
    __init_commands(){
    //define all top commands panel
        let comms = [];
        let this_ref = this;
        comms.push({testo:"Add circle", img:"../../../src_JS/image/circles.svg", action:()=> {keep_doing_action(()=>this_ref.__add_circle());}, popup:"Left click to place and size new obstacles. Right click to stop adding futher obstacles. You can delete an already place obstacle by right clicking it." });
        comms.push({testo:"Add robot", img:"image/robots.svg", action:()=>this_ref.__add_robot(), popup:"Left click to place the position of a new robot base. Then keep left clicking to set the links sizes (stop adding link by right clicking). After creation you can access the robot pendant by left clicking the robot base. The pendant is draggable. Go to the top panel of the pendant for more instructions. "});
        comms.push({testo:"Check collision", img:"image/check_coll.svg", action:function(){ 
            this_ref.__send_command("check_coll" , JSON.stringify(this_ref.__get_as_JSON()), (resp)=>{ 
                if(resp == "1")   alert("collision detected");
                else              alert("no collision present");
            }); 
        }, popup:"Check whether in the current configuration are present some collisions (all robots are checked)."});
        if(use_debug_bubble){
            comms.push({testo:"Debug bubble", img:"image/debug_bubble.svg", action:function(){
                let Q_set = this_ref.__get_Q_target();
                if(Q_set == null) { alert("at least a robot misses the target pose"); return; };
                this_ref.__send_command("ext_bubble", JSON.stringify(this_ref.__get_as_JSON()), (Q_resp)=>{
                    if(Q_resp == "null"){ console.log("extension not possible"); return; }
                    let Q_bubble = JSON.parse(Q_resp);
                    for(let r=0; r<this_ref.__robots.length; r++){
                        rad_2_grad(Q_bubble[r]);
                        this_ref.__robots[r].robot.set_pose(Q_bubble[r]);
                    }
                });
            }, popup:"Debug of bubble extension"});
        }
        comms.push({testo:"Compute path", img:"image/RRT.svg", action:function(){
            let Q_set = this_ref.__get_Q_target();
            if(Q_set == null) { alert("at least a robot misses the target pose"); return; };
            let descr = this_ref.__get_as_JSON();
            let sets = this_ref.__settings.get_fields();
            descr["params"] = [sets["det_coeff"], sets["multiple_steer"] ,sets["iterations"] ,sets["Thread"] ];
            this_ref.__send_command("plan", JSON.stringify(descr), (Q_resp)=>{
                if(Q_resp == "null"){ console.log("solution not found"); return; }
                this_ref.__add_motion_chain(JSON.parse(Q_resp));
            }); 
        }, popup:"Compute a path toward the actual target configuration (use the pendants for setting each robot target) using RRT*. Use the last button on the right to change the solver options."});
        comms.push({testo:"Profile solvers", img:"image/RRT.svg", action:function(){
            let Q_set = this_ref.__get_Q_target();
            if(Q_set == null) { alert("at least a robot misses the target pose"); return; };
            let descr = this_ref.__get_as_JSON();
            let sets = this_ref.__settings.get_fields();
            let stratgy = null;
            if(sets["strategy"] == "single") stratgy = 0;
            else if(sets["strategy"] == "bidir") stratgy = 1;
            else stratgy = 2;
            descr["params"] = [[sets["det_coeff"], sets["multiple_steer"] ,sets["iterations"] ,stratgy,sets["Trials"],sets["reallign_coeff"]] ,
                           JSON.parse(sets["Threads"])];
            this_ref.__send_command("prof", JSON.stringify(descr), (Q_resp)=>{ 
                console.log("profiling terminated");
                open_in_new_tab("profile.html"); 
            }); 
        }, popup:"The path toward the actual target (check Compute path) is compute multiple times, using the possible solvers contained in MT_RRT and different number of threads (may take a while). Results are showed in a new window (your browser may prevent the opening of new tabs and you have to manually allow it). Use the last button on the right to change the solver options. "});
        comms.push({testo:"Export as JSON", img:"../../../src_JS/image/exp.svg", action:()=>{ this_ref.__send_command("exp" , JSON.stringify(this_ref.__get_as_JSON())); }, popup:"Export the actual scene in a .json file. The pop-up allowing you to browse your folders may be opened not in foreground: minimize all the windows to find it."});
        comms.push({testo:"Import from JSON", img:"../../../src_JS/image/imp.svg", action:()=>{ this_ref.__send_command("imp" , "null", (resp)=>{ this_ref.__set_from_JSON(resp); }); }, popup:"Import a scene from a .json file. The pop-up allowing you to browse your folders may be opened not in foreground: minimize all the windows to find it."});
        comms.push({testo:"Settings", img:"../../../src_JS/image/set.svg",action:null , popup:"Open a panel for setting the options. planner:Thread is the number of threads exploited when using Compute path, while profiler:Threads is the array of threads used to profile the solvers when calling Profile solvers."});


    //create all top commands panel
        let l = 100.0 / (1.0 * comms.length);
        for(let k=0; k<comms.length; k++) {
            let txt = document.createElement("font");
            txt.style.height = "30%";
            txt.style.width = "100%";
            txt.setAttribute("size", "2%");
            txt.setAttribute("color", "white");
            txt.innerHTML = comms[k].testo;

            let img = SVG_image.get_div_image(comms[k].img);
            img.__SVG_frame.setAttributeNS(null, "height" , "70%");
            img.__SVG_frame.setAttributeNS(null, "preserveAspectRatio" , "xMidYMid meet");

            let box = document.createElement("div");
            set_bord(box);
            opacity_mouseover(box, 0.5);
            box.style.height = "100%";
            box.style.width = l + "%";
            box.style.cursor = "pointer";
            box.style.textAlign = "center";
            box.addEventListener("click", comms[k].action);
            this.__div.childNodes[0].appendChild(box);
            box.appendChild(txt);
            box.appendChild(img.__SVG_frame);

            add_popup(box, comms[k].popup, null);
        }
    }
    __init_settings(){
        let contents = [
            {name:"general", 
            panels:[{name:"det_coeff", val0:0.2, num:[0,0.5]},
                    {name:"multiple_steer", val0:1, num:null},
                    {name:"iterations", val0:2000, num:null}]
            },
            {name:"planner", 
            panels:[{name:"Thread", val0:4, num:null}]},
            {name:"profiler", 
            panels:[{name:"stratgy", val0:"star", cat:["single","bidir","star"]},
                    {name:"Trials", val0:5, num:[3,20]},
                    {name:"reallign_coeff", val0:0.1, num:[0.1, 0.5]},
                    {name:"Threads", val0:"[2,3,4]"}]
            },
        ];

        let N_comm = this.__div.childNodes[0].childNodes.length;
        this.__settings = new Setting_div(this.__div.childNodes[0].childNodes[N_comm-1], contents, 300, 400);
    };

// obstacles ///////////////////////////////////////
    __add_circle(){
        let this_ref = this;
        let new_circle = this.__create_circle(0 , 0, "0.2%");
        return new Promise((res)=>{
        // circle center
            function move_center(p){
                let p_canvas = this_ref.__canvas.pos_in_canvas(p);
                new_circle.set("cx", p_canvas[0]);
                new_circle.set("cy", p_canvas[1]);
            }
            Interaction((p)=>{
                move_center(p);
            }, (p)=>{
                move_center(p);
            }, res);
        }).then(()=>{
        // circle ray
            function set_ray(p){
                let p_canvas = this_ref.__canvas.pos_in_canvas(p);
                let delta = [p_canvas[0] -new_circle.get("cx") ,p_canvas[1] - new_circle.get("cy")];
                new_circle.set("r", Math.sqrt(delta[0]*delta[0] + delta[1]*delta[1]));
            }
            return new Promise((res)=>{
                Interaction((p)=>{
                    set_ray(p);
                }, (p)=>{
                    set_ray(p);
                }, res);
            });
        }).then(()=>{ this_ref.__register_circle(new_circle); });
    }
    __create_circle(cx, cy, ray){
        let new_circle = new SVG_circle(ray, [cx,cy], this.__canvas);
        new_circle.fill("#ff00e1");
        new_circle.set_stroke("yellow" , "0.1%");
        return new_circle;
    }
    __register_circle(to_register){
        this.__obstacles.push(to_register);
		let this_ref = this;
        to_register.__element.oncontextmenu = (e)=>{ 
            e.preventDefault();
            to_register.detach(); 
            remove(this_ref.__obstacles , to_register);
        };
    };
    
// robots    ///////////////////////////////////////
    __add_robot(){
        let this_ref = this;
        let base_circle = new SVG_circle("0.5%", [0,0], this.__canvas);
        base_circle.fill("#00ffa2");
        base_circle.set_stroke("yellow" , "0.1%");
        let skeleton = [];

        let base = [0,0];
        let joint_centers = [];
        function add_link(){
            return new Promise((res)=>{
                let new_link = new SVG_path( SVG_path.get_linear_segments([[0,0],[1,1]]), this_ref.__canvas);
                new_link.set_dash();
                new_link.set_stroke("yellow" , "0.1%");
                skeleton.push(new_link);
                let prev_p = null;
                let new_p = null;
                if(joint_centers.length == 0) prev_p = base;
                else                          prev_p = joint_centers[joint_centers.length-1];
                function move_link(p){
                    new_p = this_ref.__canvas.pos_in_canvas(p);
                    new_link.set("d",SVG_path.get_linear_segments([[prev_p[0] , prev_p[1]],[new_p[0] , new_p[1]]]));
                }
                Interaction((p)=>{
                    move_link(p);
                }, (p)=>{
                    move_link(p);
                }, ()=>{
                    joint_centers.push([new_p[0] , new_p[1]]);
                    res();
                });
            });
        };

        return new Promise((res)=>{
        // robot base
            function move_center(p){
                let p_canvas = this_ref.__canvas.pos_in_canvas(p);
                base_circle.set("cx", p_canvas[0]);
                base_circle.set("cy", p_canvas[1]);
            }
            Interaction((p)=>{
                move_center(p);
            }, (p)=>{
                move_center(p);
            }, ()=>{
                base[0] = parseInt(base_circle.get("cx"));
                base[1] = parseInt(base_circle.get("cy"));
                res();
            });
        })
        // robot links
        .then(()=>keep_doing_action(add_link))
        .then(()=>{
            base_circle.detach();
            SVG_element.detach_all(skeleton);
            let L_Q =  Planar_Robot.compute_lenghts_and_Qo(base, joint_centers);
            this_ref.__create_register_robot(base, L_Q.lenghts, L_Q.Qo);
        });
    }
    __create_register_robot(base, lenghts, Q){
        let new_rob = new Planar_Robot_interactable(this.__canvas , lenghts, base, "#00ffa2", "yellow");
        SVG_element.move_all(2 , new_rob.__get_SVG_links());
        SVG_element.move_all(2 , new_rob.__get_SVG_joint());
        new_rob.set_pose(Q);
        let new_pnd = new Pendant_floating(new_rob);
        let temp = {robot:new_rob, pendant:new_pnd};
        this.__robots.push(temp);
		let this_ref = this;
        new_rob.__links[0].circle_SVG.__element.oncontextmenu = (e)=>{ 
            e.preventDefault();
            new_rob.detach(); 
            new_pnd.detach();
            remove(this_ref.__robots , temp);
        };
        return temp;
    }
    __add_motion_chain(Qs){
        let this_ref = this;
        this.__prm_motion = this.__prm_motion.then(()=>{ return 0; });
        let t_old = null, t_new = null;
        function get_t(){
            let t =[];
            for(let kk=0; kk<this_ref.__robots.length; kk++) t.push(this_ref.__robots[kk].robot.get_end_effector());
            return t;
        }
        let segments = [];
        function print_seg(){
            for(let kk=0; kk<t_old.length; kk++){
                let seg = new SVG_path(SVG_path.get_linear_segments([t_new[kk],t_old[kk]]) , this_ref.__canvas);
                segments.push(seg);
                seg.set_stroke("yellow", "0.2%");
            }
        }

        for(let k=0; k<Qs.length; k++){
            this.__prm_motion = this.__prm_motion.then((p)=>{
                return new Promise((res)=>{
                    setTimeout(()=>{
                        for(let i=0; i<Qs[p].length; i++) rad_2_grad(Qs[p][i]);
                        this_ref.__set_Q(Qs[p]); 
                        t_new = get_t();
                        if(t_old !== null) print_seg();

                        t_old = t_new;
                        res(p+1); 
                    } , 100);
                });
            });
        }
        this.__prm_motion = this.__prm_motion.then(()=>{
            alert("press ok to continue");
            SVG_element.detach_all(segments);
        });
    }

    __send_command(comm_name , comm_body, cllbck_end = null){
        let cmm = "{\"N\":\"";
        cmm += comm_name; 
        cmm += "\",\"B\":";
        cmm += comm_body;
        cmm += "}";
        this.__connection.command(cmm, cllbck_end );
    }
    __get_as_JSON(){
        let J={};
    //obstacles
        J["obstacles"]=[];
        for(let k=0; k<this.__obstacles.length; k++) 
            J["obstacles"].push([parseInt(this.__obstacles[k].get("cx")),parseInt(this.__obstacles[k].get("cy")),parseInt(this.__obstacles[k].get("r"))]);
    //robots
        J["robots"]=[];
        function get_robot_buffer(robot){
            let bff = [robot.__base[0], robot.__base[1]];
            let k, K = robot.get_dof();
            for(k=0; k<K; k++){ bff.push(robot.__links[k].len);}
            for(k=0; k<K; k++){ bff.push(parseInt(robot.__links[k].circle_SVG.get("r")) );}
            return bff;
        }
        for(let k=0; k<this.__robots.length; k++) 
            J["robots"].push(get_robot_buffer(this.__robots[k].robot));
    //Q current
        J["Q_curr"] = this.__get_Q_current();
    //Q target
        J["Q_trgt"] = this.__get_Q_target();
        return J;
    }
    __set_from_JSON(J_string){
    //clean the actual scene
        SVG_element.detach_all(this.__obstacles);
        for(let k=0; k<this.__robots.length; k++) this.__robots[k].robot.detach();

        let J_parsed = JSON.parse(J_string);
    //obstacles
        let obst = J_parsed["obstacles"];
        for(let k=0; k<obst.length; k++)
            this.__register_circle(this.__create_circle(obst[k][0] , obst[k][1], obst[k][2]));
    //robots and Q current
        let robs = J_parsed["robots"];
        let Q_current = J_parsed["Q_curr"];
        function get_robot_lenghts(rob_json){
            let dof = (rob_json.length - 2) / 2;
            let L =[];
            for(let k=0; k<dof; k++) L.push(rob_json[k+2]);
            return L;
        }
        for(let k=0; k<robs.length; k++){
            let L = get_robot_lenghts(robs[k]);
            this.__create_register_robot([robs[k][0], robs[k][1]] , L, Q_current[k]);
        }
    }
    __get_Q_current(){
        let Q=[];
        for(let r=0; r<this.__robots.length; r++){
            Q.push(this.__robots[r].robot.get_pose());
        }
        return Q;
    }
    __get_Q_target(){
        let Q=[];
        for(let r=0; r<this.__robots.length; r++){
            if(this.__robots[r].pendant.get_Q_trg() == null) return null;
            Q.push(this.__robots[r].pendant.get_Q_trg());
        }
        return Q;
    }
    __set_Q(Q){
        if(this.__robots.length != Q.length) throw 0;
        for(let r=0; r<this.__robots.length; r++){
            this.__robots[r].robot.set_pose(Q[r]);
        }
    }
}





function keep_doing_action(action_to_do){
    return new Promise((resg)=>{
        let life = true;
        document.oncontextmenu = (e)=>{
            e.preventDefault();
            life = false;
            document.oncontextmenu = null;
        }
        let prm = new Promise((res)=>{res();});
        add();
        function add(){
            prm = prm.then(()=> action_to_do()).then(()=>{ if(life) add(); else resg(); });
        }
    });
}
