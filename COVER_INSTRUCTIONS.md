# 📸 Instruksi Menambahkan Cover Image

## Cara Menambahkan Cover Gambar ke README

### Opsi 1: Menggunakan Canva (Recommended)

1. Buka [Canva.com](https://www.canva.com)
2. Pilih template "LinkedIn Banner" atau "YouTube Thumbnail" (ukuran 1920x1080px)
3. Gunakan design dengan tema:
   - **Background**: Gradasi biru-hijau atau putih bersih
   - **Icons**: 🏥 Hospital, 🤖 Robot, 📡 WiFi, ESP32 chip
   - **Text**: 
     - Title: "SMART NURSE CALL SYSTEM"
     - Subtitle: "Micro-ROS & IoT Healthcare Solution"
   - **Elements**: Gambar ESP32, diagram sistem, medical icons
4. Export sebagai PNG (1920x1080px)
5. Simpan dengan nama `cover.png`

### Opsi 2: Template Siap Pakai

Anda bisa menggunakan tools berikut:
- [Figma Community](https://www.figma.com/community)
- [Photopea](https://www.photopea.com) - Photoshop online gratis
- [GIMP](https://www.gimp.org) - Software gratis

### Opsi 3: Generate AI Image

Gunakan AI generator seperti:
- [DALL-E](https://openai.com/dall-e-3)
- [Midjourney](https://www.midjourney.com)
- [Bing Image Creator](https://www.bing.com/create)

**Prompt suggestion:**
```
Professional banner for medical IoT project, showing ESP32 microcontroller, 
hospital icons, ROS2 logo, wireless signals, modern gradient blue-green 
background, clean minimalist design, technology theme, 1920x1080px
```

## Cara Upload dan Menggunakan Cover

### Method 1: Direct Upload ke GitHub
1. Buka repository di GitHub
2. Klik "Add file" → "Upload files"
3. Upload file `cover.png`
4. Commit dengan pesan "Add cover image"
5. Copy URL file: `https://raw.githubusercontent.com/USERNAME/REPO/main/cover.png`
6. Ganti URL di README.md baris 2

### Method 2: Menggunakan Branch assets (Recommended)
1. Buat branch baru bernama `assets`
2. Upload `cover.png` ke branch tersebut
3. URL akan menjadi: `https://raw.githubusercontent.com/USERNAME/REPO/assets/cover.png`
4. Update URL di README.md

### Method 3: External Hosting
Upload ke:
- [Imgur](https://imgur.com)
- [imgbb](https://imgbb.com)
- GitHub Issues (drag & drop untuk instant URL)

## Quick Fix: Gunakan Gambar Sementara

Jika ingin cepat, gunakan salah satu placeholder ini di README.md:

```markdown
<!-- Opsi 1: Banner sederhana dengan teks -->
<div align="center" style="background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); padding: 60px; border-radius: 10px;">
  <h1 style="color: white; font-size: 48px;">🏥 SMART NURSE CALL SYSTEM</h1>
  <p style="color: white; font-size: 24px;">Micro-ROS & IoT Healthcare Solution</p>
</div>

<!-- Opsi 2: Shields.io banner -->
![Cover](https://via.placeholder.com/1920x400/667eea/ffffff?text=SMART+NURSE+CALL+SYSTEM)

<!-- Opsi 3: Dynamic SVG -->
<svg width="100%" height="200" xmlns="http://www.w3.org/2000/svg">
  <rect width="100%" height="100%" fill="#667eea"/>
  <text x="50%" y="50%" font-size="48" fill="white" text-anchor="middle" dy=".3em">
    🏥 SMART NURSE CALL SYSTEM
  </text>
</svg>
```

## Contoh Cover Yang Bagus

### Elemen yang harus ada:
✅ Logo/Icon proyek (🏥 + 🤖)  
✅ Judul proyek yang jelas  
✅ Subtitle/tagline singkat  
✅ Teknologi utama (ESP32, ROS2, IoT)  
✅ Warna yang eye-catching tapi profesional  
✅ Resolusi tinggi (min 1200px width)  

### Inspirasi:
- Lihat repository populer di GitHub
- Cek [Awesome README](https://github.com/matiassingers/awesome-readme)
- Browse [GitHub Topics](https://github.com/topics) untuk ide visual

---

**Tips:** Jangan lupa optimize ukuran file! Gunakan [TinyPNG](https://tinypng.com) untuk compress tanpa kehilangan kualitas.
