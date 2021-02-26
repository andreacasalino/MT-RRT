class Connection{
    constructor(port, clbk_init){
    // data ////////////////////////
        this.__key = 0;
        this.__port = port;
    ////////////////////////////////
    
        this.__init(clbk_init);
    }


    command(request , clbk_response = null){
        let new_req = new XMLHttpRequest();
        new_req.open('POST', 'http://localhost:' + this.__port, true);
        if(clbk_response !== null){
            new_req.onload = ()=>{
                clbk_response(new_req.response);
            }
        }
        let to_send = '{\"KEY\":' + this.__key + ',\"BODY\":' + request + '}';
        new_req.send(to_send);
    }

    
    __init(clbk_init){
        let key_req = new XMLHttpRequest();
        key_req.open('POST', 'http://localhost:' + this.__port, true);
        let this_ref = this;
        key_req.onload = ()=>{
            if(key_req.response === 'null') throw 0; 
            this_ref.__key = key_req.response;
            console.log("generated key: " + this_ref.__key);
            clbk_init();
        }
        key_req.send('{ \"KEY\":0}');
    }
};