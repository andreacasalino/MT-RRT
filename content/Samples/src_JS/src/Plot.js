class Axis{
    constructor(wrapper, v_min, v_max, N_values, labels = null){        
        if(v_max < v_min) throw 0;
        if(N_values <= 1) throw 1;
        if(labels !== null){ if(labels.length != N_values) throw 2; }

    // data ////////////////////////
        this.__canvas = SVG_canvas.wrap_SVG_canvas(wrapper);
        this.__levels = []; //each element contain: value, label, square, line, text
    ///////////////////////////////

        this.__init_levels(v_min, v_max, N_values,labels);
    }
    __init_levels(v_min, v_max, N_values,labels){
        let K = N_values-1;
        let delta = (v_max - v_min)/(K*1.0);
        let v = v_min;

        let lbl = null;
        let sq=null;
        let ln=null;
        let txt = null;

        if(labels === null) lbl = String(Round(v));
        else                lbl = labels[0];
        sq = new SVG_rect(1 , 1,this.__canvas , 0,0); sq.fill("yellow"); sq.detach();
        ln = new SVG_path("M 0 0 L 1 1" , this.__canvas); ln.set_stroke("yellow" , "0.1%");
        txt = new SVG_element("text", this.__canvas); txt.set("font", "8px"); txt.fill("yellow");
        this.__levels.push({value:v, label:lbl, square:sq, line:ln, text:txt});
        for(let k=0; k<K; k++){
            v += delta;
            if(labels === null) lbl = String(Round(v));
            else                lbl = labels[k+1];
            sq = new SVG_rect(1 , 1,this.__canvas , 0,0); sq.fill("yellow"); sq.detach();
            ln = new SVG_path("M 0 0 L 1 1" , this.__canvas); ln.set_stroke("yellow" , "0.1%");
            txt = new SVG_element("text", this.__canvas); txt.set("font", "8px"); txt.fill("yellow");
            this.__levels.push({value:v, label:lbl, square:sq, line:ln, text:txt});
        }
    }    
    __map(vals){ //the percentages are returned
        let prc = [];
        let N = this.__levels.length;
        let p_min = 0.5 / (N*1.0);
        let p_delta = 1.0 - 1.0 / (N*1.0);
        let delta = this.__levels[this.__levels.length-1].value - this.__levels[0].value;
        for(let k=0; k<vals.length; k++) prc.push(p_min + p_delta*(vals[k] - this.__levels[0].value)/delta);
        return prc;
    };
}


class X_Axis extends Axis{
    constructor(wrapper, v_min, v_max, N_values, labels = null){
        super(wrapper, v_min, v_max, N_values, labels);
        this.resize();
    }


    resize(){
        this.__canvas.resize();
        let c_size = this.__canvas.sizes();
        let delta_L = c_size[1] / this.__levels.length;
        let L = delta_L * 0.5;
        let txt_L = delta_L*0.5;
        if(c_size[0]*0.5 < txt_L) txt_L = c_size[0]*0.5;
        for(let k=0; k<this.__levels.length; k++){
            this.__levels[k].square.set("x",  L-txt_L*0.5);
            this.__levels[k].square.set("y",  c_size[0]*0.5);
            this.__levels[k].square.set("width", txt_L);
            this.__levels[k].square.set("height", txt_L);

            this.__levels[k].text.set("x", L-txt_L*0.5); this.__levels[k].text.set("y", c_size[0]*0.5+txt_L);
            this.__levels[k].text.__element.innerHTML = this.__levels[k].label;

            this.__levels[k].line.set("d", SVG_path.get_linear_segments([[L , 0],[L , c_size[0]*0.5]]));
            L += delta_L;
        }
    }

    map(vals){ return this.__map(vals);}
}


class Y_Axis extends Axis{
    constructor(wrapper, v_min, v_max, N_values, labels = null){
        super(wrapper, v_min, v_max, N_values, labels);
        this.resize();
    }


    resize(){
        this.__canvas.resize();
        let c_size = this.__canvas.sizes();
        let delta_L = c_size[0] / this.__levels.length;
        let L = delta_L * 0.5;
        let txt_L = delta_L*0.5;
        if(c_size[1]*0.5 < txt_L) txt_L = c_size[1]*0.5;
        for(let k=0; k<this.__levels.length; k++){
            this.__levels[k].square.set("x",  0);
            this.__levels[k].square.set("y",  L-txt_L*0.5);
            this.__levels[k].square.set("width", txt_L);
            this.__levels[k].square.set("height", txt_L);

            this.__levels[k].text.set("x", 0); this.__levels[k].text.set("y", L-txt_L*0.5);
            this.__levels[k].text.__element.innerHTML = this.__levels[this.__levels.length - 1 - k].label;

            this.__levels[k].line.set("d", SVG_path.get_linear_segments([[c_size[1]*0.5 , L],[c_size[1] , L]]));
            L += delta_L;
        }
    };
    
    map(vals){ let temp = this.__map(vals); for(let k=0; k<temp.length; k++) temp[k] = 1.0 - temp[k]; return temp;}
}

class Plot{
    constructor(values, X_labels = null, p = 0.1){
    // data ////////////////////////
        this.__plot = null;
        this.__X_axis = null;
        this.__Y_axis = null;

        this.__boxes = [];
        this.__medians = [];
    ///////////////////////////////

        this.__init_div(values, X_labels, p);
        this.__init_boxes(values);
        this.__init_resize();
        this.__plot.__SVG_frame.parentNode.onresize();
    }
    __init_div(values, X_labels, p){
        function create_div(H, W){
            let temp = document.createElement("div");
            temp.style.height = H;
            temp.style.width = W;
            return temp;
        }

        let D_main = create_div("100vh","100vw" );
        document.body.appendChild(D_main);
        D_main.style.backgroundColor =  "black";

        let D_0 = create_div((100.0*(1-p)) + "%","100%" );
        D_main.appendChild(D_0);
        D_0.style.display = "flex";
        let D_0_0 = create_div("100%" , (100.0*p) + "%");
        D_0.appendChild(D_0_0);
        let D_0_1 = create_div("100%" , (100.0*(1-p)) + "%");
        D_0.appendChild(D_0_1);

        let D_1 = create_div((100.0*p) + "%","100%" );
        D_main.appendChild(D_1);
        D_1.style.display = "flex";
        let D_1_0 = create_div("100%" , (100.0*p) + "%");
        D_1.appendChild(D_1_0);
        let D_1_1 = create_div("100%" , (100.0*(1-p)) + "%");
        D_1.appendChild(D_1_1);

        this.__plot = SVG_canvas.wrap_SVG_canvas(D_0_1);
        D_0_1.style.borderLeftWidth = "0.5px";
        D_0_1.style.borderLeftColor = "yellow";
        D_0_1.style.borderLeftStyle = "Solid";
        D_0_1.style.borderBottomWidth = "0.5px";
        D_0_1.style.borderBottomColor = "yellow";
        D_0_1.style.borderBottomStyle = "Solid";

        let ax_x = {v_min:0, v_max:0, labels:null};
        for(let l=0; l<values.length; l++){
            if(values[l].values.length > ax_x.v_max) ax_x.v_max = values[l].values.length;
        }
        if(X_labels !== null) ax_x.labels = X_labels;
        ax_x.v_max = ax_x.v_max- 1;
        ax_x['N_values']=ax_x.v_max+1;
        this.__X_axis = new X_Axis(D_1_1, ax_x.v_min, ax_x.v_max, ax_x.N_values, ax_x.labels);

        let ax_y = {N_values:6 , labels:null};
        ax_y['v_min'] = get_min(values[0].values);
        ax_y['v_max'] = get_max(values[0].values);
        for(let l=1; l<values.length; l++){
            let temp = get_min(values[l].values);
            if(temp < ax_y.v_min) ax_y.v_min = temp;
            temp = get_max(values[l].values);
            if(temp > ax_y.v_max) ax_y.v_max = temp;
        }
        this.__Y_axis = new Y_Axis(D_0_0, ax_y.v_min, ax_y.v_max, ax_y.N_values, ax_y.labels);


        for(let k=0; k<values.length; k++) {
            let med = new SVG_path("M 0 0 L 1 1", this.__plot); 
            med.set_stroke(values[k].color , "0.1%")
            this.__medians.push(med);
        }

    }
    __init_resize(){
        let this_ref = this;
        this.__plot.__SVG_frame.parentNode.onresize = ()=>{
            this.__plot.resize();
            this_ref.__X_axis.resize();
            this_ref.__Y_axis.resize();
            let c;
            for(let r=0;r<this.__boxes.length; r++){
                let median_coords = [];
                for(c=0;c<this.__boxes[r].length; c++) {
                    this.__boxes[r][c].update();
                    median_coords.push([this.__boxes[r][c].__circle.get("cx") , this.__boxes[r][c].__circle.get("cy")]);
                }
                this.__medians[r].set("d", SVG_path.get_linear_segments(median_coords));
            }

        }
    }
    __init_boxes(values){
        for(let l=0; l<values.length; l++){
            let new_r = [];
            for(let v=0; v<values[l].values.length; v++) new_r.push(new boxplt( values[l].values[v], this, v, values[l].color)); 
            this.__boxes.push(new_r);
        }
    }

    set_delta_percentile(new_delta){
        let c;
        for(let r=0;r<this.__boxes.length; r++){
            for(c=0;c<this.__boxes[r].length; c++) this.__boxes[r][c].set_delta_percentile(new_delta);
        }
    };

    get_corner(){ return this.__plot.__SVG_frame.parentNode.parentNode.parentNode.childNodes[1].childNodes[0]; }
    get_main_div(){ return this.__plot.__SVG_frame.parentNode.parentNode.parentNode; };
}


class boxplt{
    constructor(values, plot, x_position, color){
        if(values.length == 0) throw 0;
    // data ////////////////////////
        this.__values = [];
        this.__x_val = x_position;
        this.__Plot = plot;
        this.__delta = 0.5;

        this.__circle = null;
        this.__vert = null;
        this.__hrz_top = null;
        this.__hrz_btm = null;
    ///////////////////////////////
        
        this.__init_values(values);
        this.__init_elements(color);
        this.update();
    }
    __init_values(values){
        let P = values.length;
        this.__values.push(values[0]);
        let p = 1;
        let this_ref = this;
        function Insert(new_val){
            let K=this_ref.__values.length;
            for(let k=0; k<K; k++){
                if(new_val < this_ref.__values[k]){
                    this_ref.__values.splice( k, 0, new_val);
                    return;
                }
            }
            this_ref.__values.push(new_val);
        }
        for(p;p<P; p++) Insert(values[p]);
    }
    __init_elements(color){
        this.__circle = new SVG_circle("0.6%", [0,0], this.__Plot.__plot); this.__circle.fill(color);
        this.__vert = new SVG_path("M 0 0 L 1 1", this.__Plot.__plot); this.__vert.set_stroke(color , "0.1%");
        this.__hrz_top = new SVG_path("M 0 0 L 1 1", this.__Plot.__plot); this.__hrz_top.set_stroke(color , "0.1%");
        this.__hrz_btm = new SVG_path("M 0 0 L 1 1", this.__Plot.__plot); this.__hrz_btm.set_stroke(color , "0.1%");
    };
    __get_percentile(prct){
        let p = (this.__values.length-1) * prct;
        let p1 = Math.floor(p);
        if(p1 == (this.__values.length-1)) return this.__values[this.__values.length-1];
        let p2 = p1 +1;
        let alfa = p - p1;
        let delta = alfa * (this.__values[p2] - this.__values[p1] );
        return this.__values[p1] + delta;
    };


    set_delta_percentile(new_delta){
        if(new_delta > 0.5) this.__delta = 0.5;
        else if(new_delta < 0.0) this.__delta = 0.0;
        else this.__delta = new_delta;
        this.update();
    }
    update(){
        let S = this.__Plot.__plot.sizes();
        let interval = this.__Plot.__Y_axis.map([this.__get_percentile(0.5 + this.__delta),
                                                this.__get_percentile(0.5),
                                                this.__get_percentile(0.5 - this.__delta)]);
        for(let k=0; k<interval.length; k++) interval[k] = interval[k] * S[0];
        let x = this.__Plot.__X_axis.map([this.__x_val])[0] * S[1];
        let delta = 0.5 * (1.0 / (this.__Plot.__X_axis.__levels.length*1.0)) * S[1];

        this.__circle.set("cx", x);
        this.__circle.set("cy", interval[1]);

        this.__vert.set("d", SVG_path.get_linear_segments([[x , interval[0]],[x , interval[2]]]));
        this.__hrz_top.set("d", SVG_path.get_linear_segments([[x-delta*0.3 , interval[2]],[x+delta*0.3 , interval[2]]]));
        this.__hrz_btm.set("d", SVG_path.get_linear_segments([[x-delta*0.3 , interval[0]],[x+delta*0.3 , interval[0]]]));
    }
}

function get_max(values){
    let c;
    let v = values[0][0];
    for(let r=0; r<values.length; r++){
        for(c=0; c<values[r].length; c++) {
            if(values[r][c] > v) v = values[r][c];
        }
    }
    return v;
}
function get_min(values){
    let c;
    let v = values[0][0];
    for(let r=0; r<values.length; r++){
        for(c=0; c<values[r].length; c++) {
            if(values[r][c] < v) v = values[r][c];
        }
    }
    return v;
}

function Round(val){
    let V = Math.round(val*100);
    V = V /100;
    return V;
}