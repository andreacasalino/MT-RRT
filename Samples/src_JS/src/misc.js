function opacity_mouseover(element, opacitiy_level = 0.5) {
    element.addEventListener("mouseover", ()=>{ element.style.opacity = opacitiy_level; });
    element.addEventListener("mouseout", ()=>{ element.style.opacity = 1.0; });
}

function click_on_off_behaviour(trigger, on_fnct, off_fnct){
    let state = true;
    trigger.onclick = ()=>{
        if(state){
            on_fnct();
            state = false;
        }
        else{
            off_fnct();
            state = true;
        }
    }

}

function Interaction(callbck , callbck_start = null, callbck_end = null){
    let p = null;

    let regime_fnct = function(e){
        e = e || window.event;
        e.preventDefault();
        if(p === null){
            p = [e.clientX, e.clientY];
            if(callbck_start !== null) callbck_start(p);
        }
        else{
            p[0] = e.clientX;
            p[1] = e.clientY;
            callbck(p);
        }
    };
    
    document.onmousemove = regime_fnct;

    document.onmouseup = function(){
        document.onmousemove = null;
        document.onmouseup = null;
        if(callbck_end != null) callbck_end();
    };
}

function remove(Collection, to_remove){
    let S = Collection.length;
    for(let s = 0; s<S; s++){
        if(Collection[s] === to_remove){
            Collection.splice(s, 1);
            return;
        }
    }
}

function open_in_new_tab(url){
    // let temp = document.createElement("a");
    // temp.setAttribute("href", url);
    // temp.setAttribute("target", "_blank");
    // temp.click();
    window.open(url);
}

function set_bord(div, color = "black"){
    div.style.borderStyle = "solid";
    div.style.borderWidth = "1px";
    div.style.borderColor = color;
}