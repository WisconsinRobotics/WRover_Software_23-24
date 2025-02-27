const circle = document.createElement('div');
circle.setAttribute('id', 'mouseCircle');
document.body.appendChild(circle);

document.addEventListener('mousemove', (e) => {
    circle.style.left = e.pageX - 10 + 'px';
    circle.style.top = e.pageY - 10 + 'px';
});

// Add CSS to hide the cursor
const style = document.createElement('style');
document.documentElement.style.cursor = 'none';
style.innerHTML = `
    html {
        cursor: none;
        
    }
    #mouseCircle {
        position: absolute;
        width: 20px;
        height: 20px;
        background-color: red;
        border-radius: 50%;
        pointer-events: none;
    }
`;
document.head.appendChild(style);