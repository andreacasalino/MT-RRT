class handle {
    constructor(val_min, val_max, Height, Width, color_background="white", color_front="black"){
        if(val_min > val_max) throw 0;
    // data ////////////////////////
        this.__value_min = val_min;
        this.__value_max = val_max;
        this.__value = this.__value_min;
        this.__onchange = [];
        this.__canvas = null;
        this.__cursor = null;
    ////////////////////////////////

        this.__create_canvas(Height, Width, color_background, color_front);
        this.__create_decrementer(Height, color_background, color_front);
        this.__create_incrementer(Height, Width, color_background, color_front);
        this.__create_cursor(Height, color_front);
        this.__update_cursor_position();
    };


    attach(wrapper){ wrapper.appendChild(this.__canvas.__SVG_frame); };

    get_value(){ return this.__value_min + this.__value_percentage * ( this.__value_max  - this.__value_min ); }
    
    set_value(value){
        this.__value = value;
        this.__update_cursor_position();
    }

    add_onchange(cllbk_onchange){ this.__onchange.push(cllbk_onchange); }


    __create_canvas(Height, Width, color_background, color_front){
        this.__canvas = new SVG_canvas([ 0, 0, 2*Height + Width, Height]);
        this.__canvas.__SVG_frame.style.backgroundColor = color_background;
        this.__canvas.__SVG_frame.style.border = "1px";
        this.__canvas.__SVG_frame.style.borderStyle = "solid";
        this.__canvas.__SVG_frame.style.borderColor = color_front;
    }
    __create_decrementer(Height, color_background, color_front){
        let bord = new SVG_rect( Height, Height, this.__canvas);
        bord.fill(color_front);
        let arrow = new SVG_path(SVG_path.get_linear_segments([[0 , 0.5*Height], [0.5*Height, Height] ,[0.5*Height, 0], [0, 0.5*Height]]), this.__canvas);
        arrow.fill(color_background);
    //add decrementer interaction
        opacity_mouseover(bord.__element);
        let this_ref =  this;
        bord.__element.onclick = ()=>{
            let new_val = this_ref.__value - 0.05 * (this_ref.__value_max - this_ref.__value_min);
            this_ref.set_value(new_val);
        }
    }
    __create_incrementer(Height,Width,color_background, color_front){
        let bord = new SVG_rect(Height, Height, this.__canvas,  Height+Width, 0);
        bord.fill(color_front);
        let arrow = new SVG_path(SVG_path.get_linear_segments([[2*Height+Width, 0.5*Height] , [3*Height/2 +  Width, 0] , [3*Height/2 +  Width, Height] , [2*Height+Width, 0.5*Height]]), this.__canvas);
        arrow.fill(color_background);
    //add incrementer interaction
        opacity_mouseover(bord.__element);
        let this_ref =  this;
        bord.__element.onclick = ()=>{
            let new_val = this_ref.__value + 0.05 * (this_ref.__value_max - this_ref.__value_min);
            this_ref.set_value(new_val);
        }
    }
    __create_cursor(Height, color_front){
        this.__cursor = new SVG_rect( Height, Height, this.__canvas, 0, 0);
        this.__cursor.fill(color_front);
        opacity_mouseover(this.__cursor.__element);
        this.__cursor.__element.style.cursor = "e-resize";

    //add moving event
        let this_ref  = this;
        let p_old = null;
        this.__cursor.__element.addEventListener("mousedown", ()=> {
            Interaction(
                (p)=>{
                    let HL = this.__get_HW(); 
                    let delta = (p[0] - p_old) / ( HL[0] - HL[1]);
                    delta = delta * (this.__value_max - this.__value_min);
                    this_ref.__value += delta;
                    this_ref.__update_cursor_position();
                    p_old = p[0];
                },
                (po)=>{ p_old = po[0]; }
            );
        });
    }
    __update_cursor_position(){
        if(this.__value > this.__value_max) this.__value = this.__value_max;
        if(this.__value < this.__value_min) this.__value = this.__value_min;

        let perc = (this.__value - this.__value_min) / (this.__value_max - this.__value_min);
        let HL = this.__get_HW();
        let pos =  perc * ( HL[0] - HL[1]) + HL[1];
        this.__cursor.set("x",pos );

        for(let i=0; i<this.__onchange.length; i++) this.__onchange[i](this.__value);
    }
    __get_HW(){ 
        let H = parseInt(this.__canvas.__SVG_frame.getAttribute("height"));
        let L = parseInt(this.__canvas.__SVG_frame.getAttribute("width")) - 2*H;
        return [L,H];
    };
}

