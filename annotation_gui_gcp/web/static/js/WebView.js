
function initialize_event_source(source, handlers) {
    let sse = new EventSource(source);
    sse.onmessage = e => {
        console.log("Message ", e);
    };
    sse.onerror = err => {
        console.log("Event: error");
        if (this.readyState == EventSource.CONNECTING) {
          console.log(`Reconnecting (readyState=${this.readyState})...`);
        } else {
          console.log("Error has occured.");
        }
    };
    sse.open = err => {
        post_json({ event: "sync", data: {}});
    };

    handlers.forEach(handler => {
        sse.addEventListener(handler.event, e => { 
                console.log("Event ", e);
                handler.handler(JSON.parse(e.data)) 
        });
    });

    return sse;
}

function button_bind(id, callback) {
    var element = document.getElementById(id);
    element.onclick = e => { callback(id, e); };
}
function check_bind(id, callback) {
    var element = document.getElementById(id);
    element.onchange = callback;
}

function post_json(data) {
    const method = 'POST';
    const headers = {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
    };
    const body = JSON.stringify(data, null, 4);
    const url = 'postdata';
    fetch(url, { method, headers, body })
}
