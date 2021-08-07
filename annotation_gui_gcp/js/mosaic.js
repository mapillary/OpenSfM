window.addEventListener('DOMContentLoaded', onDOMLoaded);
let colbox;

function onDOMLoaded() {
    colbox = document.getElementById("nColumnsListBox");
    colbox.addEventListener('change', onColumnSelect);
}

function onColumnSelect() {
    const n_columns = colbox.value;
    grid = document.getElementById("view-grid");
    const st = "auto ".repeat(n_columns)
    grid.style.gridTemplateColumns = "auto ".repeat(n_columns)
}
