function initialize_event_source(handlers) {
    let sse = new EventSource(window.location.href + '/stream');
    sse.onmessage = e => {
        console.log("Message ", e);
    };
    sse.onerror = err => {
        console.log("Event: error");
        if (this.readyState == EventSource.CONNECTING) {
            console.log(`Reconnecting (readyState=${this.readyState})...`);
        } else {
            console.log("Error has occurred.", err);
        }
    };

    // sse.open = e => {
    //     console.log("sse opened");
    //     post_json({ event: "init", data: {} });
    // };

    handlers.forEach(handler => {
        sse.addEventListener(handler.event, e => {
            console.log("Event ", e);
            handler.handler(JSON.parse(e.data))
        });
    });

    return sse;
}

function button_bind(id, callback) {
    const element = document.getElementById(id);
    element.onclick = e => { callback(id, e); };
}
function check_bind(id, callback) {
    const element = document.getElementById(id);
    element.onchange = callback;
}

function post_json(data) {
    const method = 'POST';
    const headers = {
        'Accept': 'application/json',
        'Content-Type': 'application/json',
    };
    const body = JSON.stringify(data, null, 4);
    fetch(window.location.href + "/postdata", { method, headers, body })
}