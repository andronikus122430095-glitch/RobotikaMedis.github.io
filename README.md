# RobotikaMedis.github.io
let currentSlide = 1;
const totalSlides = 12;

// Initialize
document.addEventListener('DOMContentLoaded', function() {
    createIndicators();
    showSlide(currentSlide);
    updateNavButtons();
});

// Create slide indicators
function createIndicators() {
    const indicatorsContainer = document.getElementById('slideIndicators');
    for (let i = 1; i <= totalSlides; i++) {
        const indicator = document.createElement('div');
        indicator.classList.add('indicator');
        indicator.onclick = () => goToSlide(i);
        indicatorsContainer.appendChild(indicator);
    }
}

// Show specific slide
function showSlide(n) {
    const slides = document.querySelectorAll('.slide');
    const indicators = document.querySelectorAll('.indicator');
    
    if (n > totalSlides) {
        currentSlide = totalSlides;
    }
    if (n < 1) {
        currentSlide = 1;
    }
    
    // Hide all slides
    slides.forEach(slide => {
        slide.classList.remove('active');
    });
    
    // Remove active class from all indicators
    indicators.forEach(indicator => {
        indicator.classList.remove('active');
    });
    
    // Show current slide
    slides[currentSlide - 1].classList.add('active');
    
    // Update active indicator
    if (indicators[currentSlide - 1]) {
        indicators[currentSlide - 1].classList.add('active');
    }
    
    // Update counter
    document.getElementById('slideCounter').textContent = `${currentSlide} / ${totalSlides}`;
    
    updateNavButtons();
}

// Change slide
function changeSlide(n) {
    currentSlide += n;
    showSlide(currentSlide);
}

// Go to specific slide
function goToSlide(n) {
    currentSlide = n;
    showSlide(currentSlide);
}

// Update navigation buttons state
function updateNavButtons() {
    const prevBtn = document.getElementById('prevBtn');
    const nextBtn = document.getElementById('nextBtn');
    
    if (currentSlide === 1) {
        prevBtn.disabled = true;
    } else {
        prevBtn.disabled = false;
    }
    
    if (currentSlide === totalSlides) {
        nextBtn.disabled = true;
    } else {
        nextBtn.disabled = false;
    }
}

// Keyboard navigation
document.addEventListener('keydown', function(event) {
    if (event.key === 'ArrowLeft') {
        changeSlide(-1);
    } else if (event.key === 'ArrowRight') {
        changeSlide(1);
    } else if (event.key === 'Home') {
        goToSlide(1);
    } else if (event.key === 'End') {
        goToSlide(totalSlides);
    }
});

// Touch support for mobile
let touchStartX = 0;
let touchEndX = 0;

document.addEventListener('touchstart', function(event) {
    touchStartX = event.changedTouches[0].screenX;
}, false);

document.addEventListener('touchend', function(event) {
    touchEndX = event.changedTouches[0].screenX;
    handleSwipe();
}, false);

function handleSwipe() {
    if (touchEndX < touchStartX - 50) {
        // Swipe left
        changeSlide(1);
    }
    if (touchEndX > touchStartX + 50) {
        // Swipe right
        changeSlide(-1);
    }
}

// Fullscreen toggle (F key)
document.addEventListener('keydown', function(event) {
    if (event.key === 'f' || event.key === 'F') {
        toggleFullscreen();
    }
});

function toggleFullscreen() {
    if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen();
    } else {
        if (document.exitFullscreen) {
            document.exitFullscreen();
        }
    }
}
