import { defineConfig } from 'vite';
import { serviceWorkerPlugin } from './build/service-worker-plugin';

export default defineConfig({
  base: './',
  plugins: [serviceWorkerPlugin()],
  build: {
    target: 'es2022',
    outDir: 'dist',
    assetsDir: 'assets',
    sourcemap: true,
  },
  server: {
    port: 5173,
    host: true,
  }
});
