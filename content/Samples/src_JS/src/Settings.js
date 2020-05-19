class Setting_div{
    constructor(activator, contents, Width, Height, color_header="black", color_body="white"){
    // data ////////////////////////
        this.__flt = null
        this.__fields = {};
        this.__activating_div;
    ////////////////////////////////

        this.__init_categories(contents, Width, Height, color_header, color_body);
        this.__init_activation(activator);

    }


    get_fields(){ return this.__fields; };


    __init_categories(contents, Width, Height, color_header, color_body){
        let N_div = 1;
        for(let i=0; i<contents.length; i++) N_div += contents[i].panels.length + 1;
        let h = 1.0 / (N_div *1.0);
        this.__flt = new Floating_div(Width, Height, Height * h, color_header, color_body);
        this.__flt.__deactivate_move_interaction();
        let W = document.body.getBoundingClientRect().width;
        this.__flt.__div.style.left = 0.5*(W-Width) + "px";

        let cl_txt = get_txt("Close", color_body, "center", "100%");
        this.__flt.get_header().appendChild(cl_txt); 
        opacity_mouseover(this.__flt.get_header());

        for(let k=0; k<contents.length; k++){
            let Section = document.createElement("div");
            Section.style.width = "auto";
            Section.style.height= "auto";
            this.__flt.get_body().appendChild(Section);
            Section.style.borderTopStyle = "solid";
            Section.style.borderTopWidth = "1px";
            Section.style.borderTopColor = color_header;

            let head = get_div(Height, h);
            Section.appendChild(head);
            head.appendChild(get_txt(contents[k].name, color_header, "left", "100%"));            
            for(let kk=0; kk<contents[k].panels.length; kk++){
                let pan = get_div(Height, h);
                Section.appendChild(pan);
                let N = contents[k].panels[kk].name;
                this.__fields[N] = contents[k].panels[kk].val0;

                let pan_txt = get_txt(N + ": " + this.__fields[N], color_header, "left", "70%");
                pan.appendChild(pan_txt);
                pan_txt.style.marginTop = "5%";

                let pan_btn = document.createElement("div");
                pan.appendChild(pan_btn);
                pan_btn.style.height = "100%";
                pan_btn.style.width = "30%";
                pan.appendChild(pan_btn);

                let btn = document.createElement("div");
                pan_btn.appendChild(btn);
                btn.style.height = "80%";
                btn.style.width = "80%";
                btn.style.margin = "5%";
                btn.style.backgroundColor = color_header;
                opacity_mouseover(btn);

                let set_txt = get_txt("set", color_body, "center", "100%");
                btn.appendChild(set_txt); 

            // set interaction
                if('cat' in contents[k].panels[kk])           this.__init_categoric(pan , contents[k].panels[kk], this.__fields[N]);
                else if('num' in contents[k].panels[kk])        this.__init_numeric(pan , contents[k].panels[kk], this.__fields[N]);
                else                                            this.__init_unspecified(pan , contents[k].panels[kk], this.__fields[N]);
            }
        }
    }
    __init_categoric(pan, info){
        let this_ref = this;
        pan.childNodes[1].childNodes[0].onclick = ()=>{
            let old_content = [];
            for(let k=0; k<this_ref.__flt.get_body().childNodes.length; k++){ old_content.push(this_ref.__flt.get_body().childNodes[k]);}
            this_ref.__flt.get_body().innerHTML = "";
            
            let N = info.cat.length + 1;
            let H = this_ref.__flt.get_body().getBoundingClientRect().height, h = 1.0 / (N*1.0) ;
            let squares_selector = null;
            let squares = [];
            for(let n=1;n<N; n++){
                let d = get_div(H,h);
                this_ref.__flt.get_body().appendChild(d);

                let div_square = document.createElement("div");
                d.appendChild(div_square);
                div_square.style.width = "10%";
                div_square.style.height = "100%";
                div_square.style.alignContent = "center";

                let square = document.createElement("div");
                div_square.appendChild(square);
                squares.push(square);
                square.style.width = "10px";
                square.style.height = "10px";
                square.style.marginTop = "5%";
                square.onclick = ()=>{
                    squares[squares_selector].style.backgroundColor = this_ref.__flt.get_body().style.backgroundColor;
                    squares_selector = n-1;
                    squares[n-1].style.backgroundColor = this_ref.__flt.get_header().style.backgroundColor;

                    this_ref.__fields[info.name] = info.cat[n-1];
                    pan.childNodes[0].childNodes[0].innerHTML = info.name + ":" + this_ref.__fields[info.name];
                }

                let opt_txt = get_txt(info.cat[n-1], this_ref.__flt.get_header().style.backgroundColor, "left", "70%");
                d.appendChild(opt_txt);
                opt_txt.style.marginTop = "5%";

                square.style.borderStyle = "solid";
                square.style.borderColor = this_ref.__flt.get_header().style.backgroundColor;
                square.style.borderWidth = "1px";
                if(info.cat[n-1] === this_ref.__fields[info.name]) {
                    squares_selector = n-1;
                    square.style.backgroundColor = this_ref.__flt.get_header().style.backgroundColor;
                }
            }
            let done_div = get_div(H,h);
            this_ref.__flt.get_body().appendChild(done_div);
            let done_btn = document.createElement("div");
            done_div.appendChild(done_btn);
            done_btn.style.height = "100%";
            done_btn.style.width = "50%";
            done_btn.style.marginLeft = "50%";
            done_btn.style.backgroundColor = this_ref.__flt.get_header().style.backgroundColor;
            let done_txt = get_txt("Done", this_ref.__flt.get_body().style.backgroundColor, "center", "100%");
            done_btn.appendChild(done_txt); 

            opacity_mouseover(done_btn);
            done_btn.onclick = ()=>{
                this_ref.__flt.get_body().innerHTML = "";
                for(let k=0; k<old_content.length; k++){ this_ref.__flt.get_body().appendChild(old_content[k]);}
            }
        }
    };
    __init_numeric(pan, info){
        let this_ref = this;
        pan.childNodes[1].childNodes[0].onclick = ()=>{
            let inp = prompt("enter the new value" , this_ref.__fields[info.name]);
            if(inp == null) return;
        //check is a valid number
            if(isNaN(inp) == true) return;
        //clip values if need
            if(info.num !== null){
                if(inp < info.num[0]) inp = info.num[0];
                if(inp > info.num[1]) inp = info.num[1];
            }

            this_ref.__fields[info.name] = parseFloat(inp);
            pan.childNodes[0].childNodes[0].innerHTML = info.name + ":" + inp;
        }
    };
    __init_unspecified(pan, info){
        let this_ref = this;
        pan.childNodes[1].childNodes[0].onclick = ()=>{
            let inp = prompt("enter the new value" , this_ref.__fields[info.name]);
            if(inp != null){
                this_ref.__fields[info.name] = inp;
                pan.childNodes[0].childNodes[0].innerHTML = info.name + ":" + inp;
            }
        }
    };
    __init_activation(activator){
        this.__flt.detach();
        let this_ref = this;
        function activate(){
            this_ref.__flt.attach(); 
            activator.onclick = null;
        }
        activator.onclick = activate;
        this.__flt.get_header().onclick = ()=>{
            this_ref.__flt.detach();
            activator.onclick = activate;
        }
    }
}

function get_div(Height, h){
    let D = document.createElement("div");
    D.style.width = "100%";
    D.style.height = (Height * h) + "px";
    D.style.display = "flex";
    return D;
}

function get_txt(testo, color, aling_opt, W){
    let D = document.createElement("div");
    D.style.height = "100%";
    D.style.width = W;
    D.style.textAlign = aling_opt;

    let temp = document.createElement("font");
    temp.style.color = color;
    temp.style.height = "auto";
    temp.style.Width = "auto";
    temp.innerHTML = testo;
    D.appendChild(temp);

    return D;
}