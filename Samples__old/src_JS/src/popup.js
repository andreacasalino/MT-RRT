function add_mouseover_popup(trigger, message, size = null, div_style_setter = null){

    let D = null;
    trigger.addEventListener("mouseover", ()=>{
        let trigger_client = trigger.getBoundingClientRect(); 

        D = document.createElement("div");
        D.style.height = "auto";
        if(size === null) D.style.width = trigger_client.width + "px";
        else              D.style.width = size + "px";
        D.style.position = "absolute";
    
        D.innerHTML = message;
        document.body.appendChild(D);
    
    
        let T, L, W = D.getBoundingClientRect().width, H = D.getBoundingClientRect().height;
        T = (trigger_client.top + trigger_client.height);
        L = trigger_client.left;
    
        let wndw_client = document.body.getBoundingClientRect();
        if((L+W) > (wndw_client.left + wndw_client.width)){
            let delta = (L+W) - (wndw_client.left + wndw_client.width);
            L = L - delta;
        }
        if((T+H) > (wndw_client.top + wndw_client.height)){
            let delta = (T+H) - (wndw_client.top + wndw_client.height);
            T = T - delta;
        }
    
        D.style.top = T + "px";
        D.style.left = L + "px";
    
    
    
        if(div_style_setter !== null) div_style_setter(D);
    });
    trigger.addEventListener("mouseout", ()=>{
        document.body.removeChild(D);
    });


}


function add_popup(trigger, message, size = null){
    add_mouseover_popup(trigger, message, size, (pop)=>{
        pop.style.backgroundColor = "C5FF2B";
    });
}