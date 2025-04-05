import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    outDir: './static/build', // Куда складывать файлы для Flask
    emptyOutDir: true, // Удалять старые файлы перед сборкой
  },
  base: 'static/build/',
  server: {
    port: 3000, // Локальный сервер для разработки
  },
});
