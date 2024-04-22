const circle = document.createElement('div');
circle.setAttribute('id', 'mouseCircle');
document.body.appendChild(circle);

document.addEventListener('mousemove', (e) => {
    circle.style.left = e.pageX -10+ 'px';
    circle.style.top = e.pageY -10+ 'px';
});