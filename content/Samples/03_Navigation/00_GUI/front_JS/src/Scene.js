const use_conn  = true;

class Scene{
    constructor(){
    // data ////////////////////////
        this.__div = null; //on top the command panel, on bottom the svg canvas
        this.__connection = null;
        this.__settings=null;
        this.__canvas = null;
        this.__obstacles = [];
        this.__Start = null
        this.__vehicle_sizes = null;
        this.__End   = null;
        this.__prm_motion = new Promise((res)=>{ res(0); });
    ///////////////////////////////

        let this_ref = this;
        new Promise((res)=>{
            if(use_conn) this_ref.__connection = new Connection( 3001, res);
            else res();
        }).then(()=>{
            this_ref.__init_head_body();
            this_ref.__init_commands();
            this_ref.__init_settings();
            this_ref.__init_configurations();
            window.onbeforeunload = ()=>{ this_ref.__send_command("kill" , "null"); }
            return new Promise((res)=>{ res(); });
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
        this.__canvas = SVG_canvas.wrap_SVG_canvas(canvas, 2);
    };
    __init_commands(){
    //define all top commands panel
        let comms = [];
        let this_ref = this;
        comms.push({testo:"Add circle", img:"../../../src_JS/image/circles.svg", action:()=> {keep_doing_action(()=>this_ref.__add_circle());} });
        comms.push({testo:"Compute path", img:"./image/RRT.svg", action:function(){
            let descr = this_ref.__get_as_JSON();
            let sets = this_ref.__settings.get_fields();
            descr["params"] = [sets["det_coeff"],sets["iterations"] ,sets["Thread"] ];
            this_ref.__send_command("plan", JSON.stringify(descr), (Q_resp)=>{
                if(Q_resp == "null"){ console.log("solution not found"); return; }
                this_ref.__draw_solution(JSON.parse(Q_resp));
            }); 
        }});
        comms.push({testo:"Profile solvers", img:"./image/RRT.svg", action:function(){
            let descr = this_ref.__get_as_JSON();
            let sets = this_ref.__settings.get_fields();
            let stratgy = null;
            if(sets["strategy"] == "single") stratgy = 0;
            else if(sets["strategy"] == "bidir") stratgy = 1;
            else stratgy = 2;
            descr["params"] = [[sets["det_coeff"] ,sets["iterations"] ,stratgy,sets["Trials"],sets["reallign_coeff"]] ,
                           JSON.parse(sets["Threads"])];
            this_ref.__send_command("prof", JSON.stringify(descr), (Q_resp)=>{ 
                console.log("profiling terminated");
                open_in_new_tab("profile.html"); 
            }); 
        }});
        comms.push({testo:"Export as JSON", img:"../../../src_JS/image/exp.svg", action:()=>{ this_ref.__send_command("exp" , JSON.stringify(this_ref.__get_as_JSON())); }});
        comms.push({testo:"Import from JSON", img:"../../../src_JS/image/imp.svg", action:()=>{ this_ref.__send_command("imp" , "null", (resp)=>{ this_ref.__set_from_JSON(resp); }); }});
        comms.push({testo:"Settings", img:"../../../src_JS/image/set.svg",action:null });


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
        }
    }
    __init_settings(){
        let contents = [
            {name:"general", 
            panels:[{name:"det_coeff", val0:0.2, num:[0,0.5]},
                    {name:"iterations", val0:2000, num:null},
                    {name:"Ray", val0:50, num:[5,300]}]
            },
            {name:"planner", 
            panels:[{name:"Thread", val0:4, num:null}]},
            {name:"profiler", 
            panels:[{name:"stratgy", val0:"star", cat:["single","star"]},
                    {name:"Trials", val0:5, num:[3,20]},
                    {name:"reallign_coeff", val0:0.1, num:[0.1, 0.5]},
                    {name:"Threads", val0:"[2,3,4]"}]
            },
        ];

        let N_comm = this.__div.childNodes[0].childNodes.length;
        this.__settings = new Setting_div(this.__div.childNodes[0].childNodes[N_comm-1], contents, 300, 400);
    };
    __init_configurations(){
        let cS = this.__canvas.sizes();
        this.__vehicle_sizes = [0.05 * cS[0] , 0.05 * cS[1]];
        this.__Start = new Configuration(0.25 * cS[1], 0.25 * cS[0], 0.0, this.__vehicle_sizes, this.__canvas, "#5eff00");
        this.__End = new Configuration(0.75 * cS[1], 0.75 * cS[0], 1.57, this.__vehicle_sizes, this.__canvas, "#ff0090");
        let this_ref = this;
        function res_evnt(){
            let new_size = prompt("Enter new sizes [Width, Length]", "[" + this_ref.__vehicle_sizes[0] + "," + this_ref.__vehicle_sizes[1] + "]");
            if (new_size != null) {
                let temp = JSON.parse(new_size);
                this_ref.__vehicle_sizes = [temp[0], temp[1] ];
                this_ref.__Start.__update_sizes(this_ref.__vehicle_sizes);
                this_ref.__End.__update_sizes(this_ref.__vehicle_sizes);
            }
        }
        this.__Start.__box.__element.oncontextmenu = (e)=>{ e.preventDefault(); res_evnt(); }
        this.__End.__box.__element.oncontextmenu = (e)=>{ e.preventDefault(); res_evnt(); }
    }

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

    __draw_solution(Q_resp){
        let path =  new SVG_path(SVG_path.get_linear_segments(Q_resp), this.__canvas);
        path.move(1);
        path.set_stroke("aqua", "0.1%");
        let vehicle = SVG_rect.get_centered([0,0] , this.__vehicle_sizes[1], this.__vehicle_sizes[0], this.__canvas);
        vehicle.move(1);
        vehicle.fill("yellow");
        for(let p=0; p<Q_resp.length; p++){
            this.__prm_motion = this.__prm_motion.then((p)=>{
                return new Promise((res)=>{
                    setTimeout(()=>{
                        vehicle.set_rot_pos(Q_resp[p][2]*180.0 / 3.141, [Q_resp[p][0] ,Q_resp[p][1] ]);
                        res(p+1);
                    },10);
                });
            });    
        }
        this.__prm_motion = this.__prm_motion.then(()=>{
            alert("solution found");
            vehicle.detach();
            path.detach();
        });
        this.__prm_motion = this.__prm_motion.then(()=>{
            return new Promise((res)=>{ res(0); });
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
    //info
        let cnvs_size = this.__canvas.sizes();
        J["info"] = [0, cnvs_size[1],0, cnvs_size[0], this.__settings.get_fields()["Ray"], this.__vehicle_sizes[0], this.__vehicle_sizes[1]];
    //Q current
        J["Q_curr"] = this.__Start.get_coordinates();
    //Q target
        J["Q_trgt"] = this.__End.get_coordinates();
        return J;
    }
    __set_from_JSON(J_string){
    //clean the actual scene
        SVG_element.detach_all(this.__obstacles);

        let J_parsed = JSON.parse(J_string);
    //obstacles
        let obst = J_parsed["obstacles"];
        for(let k=0; k<obst.length; k++)
            this.__register_circle(this.__create_circle(obst[k][0] , obst[k][1], obst[k][2]));
    //ray 
        this.__settings.get_fields()["Ray"] = J_parsed["info"][4];
    //Q current and Q target
        this.__vehicle_sizes = [J_parsed["info"][5], J_parsed["info"][6]];
        this.__Start.set_coordinates(J_parsed["Q_curr"]);
        this.__Start.__update_sizes(this.__vehicle_sizes);
        this.__End.set_coordinates(J_parsed["Q_trgt"]);
        this.__End.__update_sizes(this.__vehicle_sizes);
    }
}



class Configuration {
    constructor(x,y,angle, sizes, canvas, color){
    // data ////////////////////////
        this.__center = [x,y];
        this.__angle = angle;
        this.__sizes = [sizes[0], sizes[1]];
        this.__box = null;
        this.__versor = null;
    ///////////////////////////////

        this.__init_box(canvas, color);
        this.__init_arrow(canvas, color);
        this.set_coordinates([x,y,angle]);
    }
    __init_box(canvas, color){
        this.__box = new SVG_rect( this.__sizes[1], this.__sizes[0], canvas, -0.5*this.__sizes[1] ,-0.5*this.__sizes[0] );
        this.__box.move(1);
        this.__box.fill(color);
        this.__box.set_stroke("yellow" , "0.1%");

        let this_ref = this;
        function traslate(p){
            let p_canvas = this_ref.__box.__canvas.pos_in_canvas(p);
            let c = this_ref.get_coordinates();
            c[0] = p_canvas[0];
            c[1] = p_canvas[1];
            this_ref.set_coordinates(c);
        }
        this.__box.__element.onmousedown = ()=>{
            Interaction((p)=>{
                traslate(p);
            }, (p)=>{
                traslate(p);
            });
        }
    };
    __init_arrow(canvas){
        this.__versor = new SVG_path(SVG_path.get_linear_segments([[0,0],[this.__sizes[1],0]]) , canvas);
        this.__versor.move(1);
        this.__versor.set_stroke("yellow" , "0.5%");

        let this_ref = this;
        function rotate(p){
            let p_canvas = this_ref.__box.__canvas.pos_in_canvas(p);
            let c = this_ref.get_coordinates();
            c[2] = Math.atan2(p_canvas[1] -c[1], p_canvas[0] -c[0]);
            this_ref.set_coordinates(c);
        }
        this.__versor.__element.onmousedown = ()=>{
            Interaction((p)=>{
                rotate(p);
            }, (p)=>{
                rotate(p);
            });
        }
    };
    __update_sizes(new_sizes){
        this.__sizes = [new_sizes[0], new_sizes[1]];
        this.__box.set("x" , -0.5*this.__sizes[1]);
        this.__box.set("y" , -0.5*this.__sizes[0]);
        this.__box.set("width" , this.__sizes[1]);
        this.__box.set("height" , this.__sizes[0]);
        this.__versor.set("d", SVG_path.get_linear_segments([[0,0],[this.__sizes[1],0]]));
    };


    get_coordinates(){ return [this.__center[0], this.__center[1] , this.__angle]; };
    
    set_coordinates(coords){
        this.__center = [coords[0], coords[1]];
        this.__angle = coords[2];
        this.__box.set_rot_pos(this.__angle*180.0 / 3.141, this.__center);
        this.__versor.set_rot_pos(this.__angle*180.0 / 3.141, this.__center);
    };
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
