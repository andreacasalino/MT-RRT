class Floating_div{
    constructor(Width, Height, Height_header, color_header="black", color_body="white"){ //Height total
        if(Height_header > Height) throw 0;
    // data ////////////////////////
        this.__attached = false;
        this.__cllb_onmousemove = null; ///
        this.__div = null;    
    ////////////////////////////////

        this.__create_head_and_body( Width, Height, Height_header, color_header, color_body);
        this.__init_move_interaction();
        this.attach();
    };


    attach(){
        if(this.__attached) throw 0;
        document.body.appendChild(this.__div);
        this.__attached =  true;
    };

    detach(){
        if(!this.__attached) throw 0;
        document.body.removeChild(this.__div);
        this.__attached = false;
    };

    set_onmove(cllb){ this.__cllb_onmousemove = cllb; }

    get_header(){ return this.__div.childNodes[0]; }
    
    get_body(){ return this.__div.childNodes[1]; }


    __create_head_and_body(Width, Height, Height_header, color_header, color_body){
        this.__div = document.createElement("div");
        this.__div.style.width = Width + "px";
        this.__div.style.height = "auto";
        this.__div.style.position = "absolute";
        this.__div.style.left = "0px";
        this.__div.style.top = "0px";        
        this.__div.style.border = "1px";
        this.__div.style.borderStyle = "solid";
        this.__div.style.borderColor = color_header;

        let header = document.createElement("div");
        this.__div.appendChild(header);
        header.style.width = "100%";
        header.style.height = Height_header + "px";
        header.style.backgroundColor = color_header;

        let body = document.createElement("div");
        this.__div.appendChild(body);
        body.style.width = "100%";
        body.style.height = (Height - Height_header) + "px";
        body.style.backgroundColor = color_body;
    };
    __init_move_interaction(){
        let p_old = null;
        let this_ref = this;
        this.__div.childNodes[0].style.cursor  = "move";
        this.__div.childNodes[0].onmousedown = function(){
            Interaction(
                function(p){
                    let t = [p[0] - p_old[0] , p[1] - p_old[1]];
                    p_old = [p[0], p[1]];
                    this_ref.__div.style.left = (this_ref.__div.offsetLeft + t[0]) + "px";
                    this_ref.__div.style.top = (this_ref.__div.offsetTop + t[1]) + "px";
                    if(this_ref.__cllb_onmousemove !== null) this_ref.__cllb_onmousemove(p);
                },
                function(p){
                    p_old = [p[0], p[1]];
                },
                null
            );
        }
    }
    __deactivate_move_interaction(){ 
        this.__div.childNodes[0].style.cursor  = "default";
        this.__div.childNodes[0].onmousedown = null; 
    }
}