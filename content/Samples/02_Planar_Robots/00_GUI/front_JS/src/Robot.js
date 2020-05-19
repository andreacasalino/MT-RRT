class Planar_Robot {
    constructor(svg_canvas , lengths, base, color_fill, color_border, q_min= null, q_max = null){
        let L = lengths.length;
        if(L == 0) throw 0;
        for(let l=0; l<L; l++){
            if(lengths[l] <= 0.0) throw 0;
        }
        if(q_min !== null){
            if(L != q_min.length) throw 0;
        }
        if(q_max !== null){
            if(L != q_max.length) throw 0;
        }
    // data ////////////////////////
        this.__canvas = svg_canvas;
        this.__attached = true;
        this.__base = [base[0], base[1]];
        this.__links = []; //each element has: circle_SVG, link_SVG, len, q_angle, p_cartesian
        this.__onchange = [];
        this.__q_min = [];
        this.__q_max = [];
    ////////////////////////////////

        this.__compute_links(svg_canvas , lengths, color_fill, color_border);
        this.__init_min_max(q_min, q_max);
    };


    attach(){
        if(this.__attached) throw 0;
        let shapes = this.__get_SVG_shapes();
        for(let i=0;i<shapes.length;i++) shapes[i].attach(this.__canvas);
    }

    detach(){
        if(!this.__attached) throw 0;
        let shapes = this.__get_SVG_shapes();
        for(let i=0;i<shapes.length;i++) shapes[i].detach();
    }

    set_pose(Q){ //Q must be degree (0-360)
        if(Q.length != this.__links.length) throw 0;
        let q_cum = 0;
        let p = [this.__base[0] , this.__base[1]];
        for(let i=0; i<this.__links.length; i++){
            this.__links[i].q_angle = Q[i];
            if(this.__links[i].q_angle > this.__q_max[i]) this.__links[i].q_angle = this.__q_max[i];
            if(this.__links[i].q_angle < this.__q_min[i]) this.__links[i].q_angle = this.__q_min[i];
            q_cum = q_cum + this.__links[i].q_angle;
            if(q_cum > 180) q_cum = q_cum - 360;
            if(q_cum < -180) q_cum = q_cum + 360;  
            this.__links[i].circle_SVG.set_rot_pos(q_cum, p); 
            this.__links[i].link_SVG.set_rot_pos(q_cum, p);
            p[0] = p[0] + this.__links[i].len * Math.cos(q_cum * 3.14159 / 180.0);
            p[1] = p[1] + this.__links[i].len * Math.sin(q_cum * 3.14159 / 180.0);
            this.__links[i].p_cartesian = [p[0], p[1]];
        }

        for(let i=0; i<this.__onchange.length; i++) this.__onchange[i](Q);
    };

    set_pose_single(q, j){
        let Q = this.get_pose();
        Q[j] = q;
        this.set_pose(Q);
    }

    get_pose(){
        let Q=[];
        for(let i=0; i<this.__links.length; i++){
            Q.push(this.__links[i].q_angle);
        }
        return Q;
    };

    get_dof(){ return this.__links.length; }

    get_lengths(){
        let L =[];
        for(let k=0; k<this.__links.length; k++){
            L.push(this.__links[k].len);
        }
        return L;
    }

    get_base(){ return [this.__base[0], this.__base[1]]; }

    get_end_effector(){ 
        let p = this.__links[this.__links.length-1].p_cartesian;
        return [p[0],p[1]]; 
    }

    static compute_lenghts_and_Qo(base, joint_positions){ //joit positions is an array with the 2D coordinates of evry point in the kinematic chain
        if(joint_positions.length == 0) throw 0;
        let J = joint_positions.length;
        let delta_abs, delta;
        let M_prev = {R:[[1,0],[0,1]], T:base};
        let c_prev = base;
        let l, lenghts = [], Qo=[];
        for(let j=0; j<J; j++){
            delta_abs = [joint_positions[j][0] -c_prev[0] , joint_positions[j][1] -c_prev[1]];
            l = Math.sqrt( delta_abs[0]*delta_abs[0] + delta_abs[1]*delta_abs[1] );
            lenghts.push(l);

            delta = [M_prev.R[0][0] * delta_abs[0] +M_prev.R[1][0] * delta_abs[1],
                     M_prev.R[0][1] * delta_abs[0] +M_prev.R[1][1] * delta_abs[1]];
            Qo.push(Math.atan2(delta[1] ,delta[0] ));

            M_prev.T = sum(product_Mat_Vec(M_prev.R, [l,0]), M_prev.T);
            let Cq = Math.cos(Qo[j]);
            let Sq = Math.sin(Qo[j]);
            M_prev.R = product_Mat_Mat(M_prev.R , [[Cq, -Sq],[Sq, Cq]]);
            c_prev = joint_positions[j];

            Qo[j] = Qo[j] * 180.0 / 3.14159;
        }
        return {lenghts:lenghts, Qo:Qo};
    };
    
    add_onchange(cllbk_change){ this.__onchange.push(cllbk_change); };


    __compute_links(svg_canvas, lengths, color_fill, color_border){
        let L = 0;
        for(let l=0; l<lengths.length; l++){
            let temp_circ = new SVG_circle(1, [0,0], svg_canvas);
            temp_circ.set_stroke(color_border, "0.2%");
            temp_circ.fill(color_fill);
            temp_circ.detach();
            let temp_link = new SVG_path("M 0 0 L 1 1" , svg_canvas);
            temp_link.set_stroke(color_border, "0.2%");
            temp_link.fill(color_fill);
            let temp =  {circle_SVG:temp_circ, link_SVG:temp_link,  len : lengths[l], q_angle:0, p_cartesian:[0,0] };
            this.__links.push(temp);
            L = L + temp.len;
        }
        let rays = [];
        let coeff = 1.0/30.0;
        let l_old = L *coeff , l_att, L_cum=0;
        let Qo = [];
        for(let l=0; l<lengths.length; l++ ){
            L_cum = L_cum + lengths[l];
            l_att = coeff * L * (1 - L_cum / L);
            this.__links[l].link_SVG.set("d", "M 0 0 L 0 " + l_old + " L " + lengths[l] + " " + l_att + " L " +lengths[l] + " -" + l_att + " L 0 -" + l_old + " L 0 0" );
            rays.push(l_old * 1.5);
            l_old = l_att;
            Qo.push(0.0);
            this.__links[l].circle_SVG.attach(this.__canvas);
        }
        for(let l=0; l<lengths.length; l++ ) this.__links[l].circle_SVG.set("r" , rays[l]);
        this.set_pose(Qo);
    }
    __get_SVG_shapes(){
        let I = this.get_dof();
        let shapes = [];
        for(let i=0; i<I; i++){
            shapes.push(this.__links[i].circle_SVG);
            shapes.push(this.__links[i].link_SVG);
        }
        return shapes;
    }
    __get_SVG_links(){
        let I = this.get_dof();
        let shapes = [];
        for(let i=0; i<I; i++) shapes.push(this.__links[i].link_SVG);
        return shapes;
    }
    __get_SVG_joint(){
        let I = this.get_dof();
        let shapes = [];
        for(let i=0; i<I; i++) shapes.push(this.__links[i].circle_SVG);
        return shapes;
    }
    __init_min_max(q_min, q_max){
        for(let i=0; i<this.__links.length; i++){
            if(q_min == null) this.__q_min.push(-180);
            else this.__q_min.push(q_min[i]);
            if(q_max == null) this.__q_max.push(180);
            else this.__q_max.push(q_max[i]);
        }
    }
}



class Planar_Robot_interactable extends Planar_Robot {
    constructor(svg_canvas , lengths, base, color_fill, color_border){
        super(svg_canvas , lengths, base, color_fill, color_border);

        this.__add_angular_motion();
        this.__add_cartesian_motion();
    };


    __add_angular_motion(){ 
        let this_ref = this;
        for(let l=0; l<this.__links.length; l++){
            let c,  p_old = [0,0];
            this.__links[l].link_SVG.__element.onmousedown = function(){
                Interaction(
                    function(p){
                        let P_cnv = this_ref.__canvas.pos_in_canvas(p);
                        let alfa2 = Math.atan2(P_cnv[1] - c[1] ,P_cnv[0] - c[0]);
                        let alfa1 = Math.atan2(p_old[1] - c[1] ,p_old[0] - c[0]);
                        p_old = [P_cnv[0], P_cnv[1]];
                        this_ref.set_pose_single(this_ref.__links[l].q_angle + (alfa2 - alfa1)*180.0 / 3.14159  , l);
                    },
                    function(p){
                        if(l == 0) c = this_ref.__base;
                        else       c = this_ref.__links[l-1].p_cartesian;
                        p_old = this_ref.__canvas.pos_in_canvas(p);
                    }
                );
            };
        }
    };
    __add_cartesian_motion(){
        let this_ref = this;
        let L =this.__links.length;
        for(let l=0; l<(L-1); l++ ){
            this.__links[l+1].circle_SVG.__element.addEventListener("click", function(){ this_ref.__cartesian_motion(l, true); });
        }
        let ll = L - 1;
        this.__links[L - 1].link_SVG.__element.addEventListener("click" , function(){ this_ref.__cartesian_motion(ll, true);} );
    };
    __cartesian_motion(pos_chain, use_pseudo){
        let to_highligh = null;
        let act_pos = this.__links[pos_chain].p_cartesian;
        if(pos_chain != (this.__links.length - 1)) to_highligh = this.__links[pos_chain+1].circle_SVG;
        else                                       to_highligh = this.__links[this.__links.length - 1].link_SVG; 

        to_highligh.set("stroke-width" , "0.4%");

        let target = [0,0];
        let this_ref = this;
        let terminate = false;
        function continuous_motion(){
            setTimeout(
                ()=>{ 
                this_ref.__moveto(target , pos_chain, use_pseudo);
                if(!terminate) continuous_motion(); }
                , 250);
        }
        let line = null;
        Interaction(
            function(p){
                target = this_ref.__canvas.pos_in_canvas(p);
                line.set("d" , SVG_path.get_linear_segments([[act_pos[0] , act_pos[1]], [target[0],target[1] ]]));
            },
            function(p){
                target = this_ref.__canvas.pos_in_canvas(p);
                line = new SVG_path(SVG_path.get_linear_segments([[0,0],[1,1]]), this_ref.__canvas);
                line.set_stroke(this_ref.__links[0].circle_SVG.get("stroke") , "0.1%");
                line.set_dash();
                terminate = false;
                continuous_motion();
            },
            function(){
                line.detach();
                terminate = true;
                to_highligh.set("stroke-width" , "0.2%");
            }
        );
    }
    __get_J(pos_chain){
        let J =[[],[]];
        let k;
        for(k = 0; k< this.get_dof(); k++){
            J[0].push(0);
            J[1].push(0);
        }
        let c = this.__base;
        let E = this.__links[pos_chain].p_cartesian;
        for(k = 0; k<= pos_chain; k++){
            J[0][k] = -(E[1] - c[1]);
            J[1][k] =  (E[0] - c[0]);
            c = this.__links[k].p_cartesian;
        }
        return J;
    };
    __moveto(x_set, pos_chain, use_pseudo){
        let errX = [0.1 * (x_set[0] - this.__links[pos_chain].p_cartesian[0]) ,0.1 * (x_set[1] - this.__links[pos_chain].p_cartesian[1]) ] ;
        let Jmove = null;
        if(use_pseudo){
            try{ Jmove = get_psinvJ(this.__get_J(pos_chain));}
            catch(err){ return; };
        } 
        else           Jmove = get_transpose(this.__get_J(pos_chain));
        let delta_Q = product_Mat_Vec(Jmove, errX);
        if(pos_chain != (this.get_dof() - 1)){
            let Jproj = null;
            let Jtemp = this.__get_J(this.__links.length - 1);
            if(use_pseudo){
                try{ Jproj = get_psinvJ(Jtemp);}
                catch(err){ return; };
            } 
            else             Jproj = get_transpose(Jtemp);
            Jproj = product_Mat_Mat( Jproj, Jtemp);
            let deltaQ2 = product_Mat_Vec(Jproj , delta_Q);
            delta_Q = diff(delta_Q, deltaQ2);
        }
        rad_2_grad(delta_Q);
        let Q_new = sum( this.get_pose(), delta_Q);
        this.set_pose(Q_new);
    }
}
