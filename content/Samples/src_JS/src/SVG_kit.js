class SVG_element{
    constructor(name, SVG_wrapper){
    // data ////////////////////////
        this.__element = document.createElementNS("http://www.w3.org/2000/svg",name);
        this.__rotation = 0;
        this.__traslation = [0, 0];
        this.__canvas = null;
        this.__canvas_layer = null;
    ///////////////////////////////
        if(SVG_wrapper !== null) this.attach(SVG_wrapper);
    };


    set(name , value){
        this.__element.setAttributeNS(null, name, value);
    };

    get(name){
        //return this.__element.getAttributeNS( "http://www.w3.org/2000/svg", name);
        return this.__element.getAttribute(name);
    }

    set_rot(angle){
        this.__rotation = angle;
        this.__set_rot_pos();
    };

    set_pos(new_pos){
        this.__traslation = [new_pos[0], new_pos[1]];
        this.__set_rot_pos();
    }

    set_rot_pos(angle, pos){
        this.__rotation = angle;
        this.__traslation = [pos[0], pos[1]];
        this.__set_rot_pos();
    };

    __set_rot_pos(){
        let temp ="translate(" + this.__traslation[0] + ", " + this.__traslation[1] + ")";
        temp = temp + " rotate(" + this.__rotation + ") ";
        this.__element.setAttributeNS(null, "transform", temp);
    }

    attach(svg_canvas, layer = 0){
        if(this.__canvas != null) throw 0;
        this.__canvas = svg_canvas;
        this.__canvas_layer = layer;
        this.__canvas.__layers[this.__canvas_layer].appendChild(this.__element);
    }

    move(layer){
        if(this.__canvas == null) throw 0;
        if(this.__canvas_layer === layer) return;
        if(layer >= this.__canvas.__layers.length) throw 0;
        this.__canvas.__layers[this.__canvas_layer].removeChild(this.__element);
        this.__canvas.__layers[layer].appendChild(this.__element);
        this.__canvas_layer = layer;
    }

    detach(){
        if(this.__canvas !== null){
            this.__canvas.__layers[this.__canvas_layer].removeChild(this.__element);
            this.__canvas = null;
            this.__canvas_layer = null;
        }
    }

    static attach_all(svg_canvas, SVG_elements, level = 0){
        for(let i=0; i<SVG_elements.length; i++) SVG_elements[i].attach(svg_canvas, level);
    }

    static move_all(level, SVG_elements){
        for(let i=0; i<SVG_elements.length; i++) SVG_elements[i].move(level);
    }

    static detach_all(SVG_elements){
        for(let i=0; i<SVG_elements.length; i++) SVG_elements[i].detach();
    }

    static set_all(name, value , SVG_elements){
        for(let i=0; i<SVG_elements.length; i++) SVG_elements[i].set(name, value);
    }

//these may have no meaning for some objects
    fill(color){ this.set("fill", color);}
   
    set_dash(){ this.set("stroke-dasharray", "3.18"); }
   
    set_stroke(color,stroke_size){
        this.set("stroke" , color);
        this.set("stroke-width" , stroke_size);
    };
}

class SVG_canvas {
    constructor(view_box, N_layer = 1){
        if(N_layer < 1) throw 0;
    // data ////////////////////////
        this.__SVG_frame = null;
        this.__layers = [];
    ////////////////////////////////

        this.__SVG_frame  = document.createElementNS("http://www.w3.org/2000/svg","svg");
        this.__SVG_frame.setAttributeNS(null, "width" , view_box[2] + "px");
        this.__SVG_frame.setAttributeNS(null, "height" , view_box[3] + "px");
        this.__SVG_frame.setAttributeNS(null, "viewBox", view_box[0] + " " + view_box[1] + " " + view_box[2]+ " " + view_box[3]);
        this.__init_layers(N_layer);
    };
    __init_layers(N_layer){
        for(let k=0; k<N_layer; k++){
            let g = document.createElementNS("http://www.w3.org/2000/svg","g");
            this.__SVG_frame.appendChild(g);
            this.__layers.push(g);
        }
    };


    static wrap_SVG_canvas(wrapper, N_layer = 1){
        if(wrapper.innerHTML !== "") throw 0;
        let canvas_BB = wrapper.getBoundingClientRect(); 
        let temp = new SVG_canvas([0,0,canvas_BB.width, canvas_BB.height], N_layer);
        temp.__SVG_frame.setAttributeNS(null, "width" , "100%");
        temp.__SVG_frame.setAttributeNS(null, "height" , "100%");
        wrapper.appendChild(temp.__SVG_frame);
        return temp;
    }

    sizes(){
        let canvas_BB = this.__SVG_frame.getBoundingClientRect(); 
        return [canvas_BB.height , canvas_BB.width]; 
    }

    resize(){
        let S =this.sizes();
        this.__SVG_frame.setAttributeNS(null, "viewBox", 0 + " " + 0 + " " + S[1]+ " " + S[0]);
    }

    attach(wrapper){ wrapper.appendChild(this.__SVG_frame); };

    pos_in_canvas(p_cursor_abs){
        let BB = this.__SVG_frame.getBoundingClientRect();
        let x = (p_cursor_abs[0] - BB.x);
        let y = (p_cursor_abs[1] - BB.y);
        return [x, y];
    }
}


class SVG_path extends SVG_element{
    constructor(d, SVG_wrapper){
        super("path", SVG_wrapper);
        this.set("d" , d);
        this.set("fill", "none");
    }

    static get_linear_segments(coordinates){
       if(coordinates.length < 2) throw 0;
        let p = "M " + coordinates[0][0] + " " + coordinates[0][1];
        for(let i=1; i<coordinates.length; i++){
            p = p + " L " + coordinates[i][0] + " " + coordinates[i][1];
        }
        return p;
    }
}

class SVG_rect extends SVG_element{
    constructor(W, H, SVG_wrapper, x = 0, y = 0){
        super("rect", SVG_wrapper);
        this.set("width", W);
        this.set("height", H);
        this.set("x", x);
        this.set("y", y);
    }

    static get_centered(center, W, H, SVG_wrapper){
        return new SVG_rect(W,H, SVG_wrapper, center[0] - W*0.5, center[1] - H*0.5 );
    }
}

class SVG_image extends SVG_element{
    constructor(x, y, W, H, relative_path, SVG_wrapper){
        super("image", SVG_wrapper);
        this.set("x", x);
        this.set("y", y);
        this.set("width", W);
        this.set("height", H);
        this.set("href", relative_path);
    }

    static get_div_image(relative_path, div_sizes = null){
        let div = new SVG_canvas([0,0,1,1]);
        div.__SVG_frame.setAttributeNS(null, "width" , "100%");
        div.__SVG_frame.setAttributeNS(null, "height" , "100%");
        div.__SVG_frame.setAttributeNS(null, "preserveAspectRatio", "none");
        new SVG_image(0,0,1,1,relative_path, div);
        if(div_sizes !== null){
            div.__SVG_frame.setAttributeNS(null, "width" , div_sizes[0] + "px");
            div.__SVG_frame.setAttributeNS(null, "height" , div_sizes[1] + "px");
        }
        return div;
    }
}

class SVG_circle extends SVG_element{
    constructor(ray, center, SVG_wrapper){
        super("circle", SVG_wrapper);
        this.set("cx", center[0]);
        this.set("cy", center[1]);
        this.set("r", ray);
    }
}