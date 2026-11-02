import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// "npm run build" ciktisi ROS paketine kopyalanir; dashboard_server onu sunar.
export default defineConfig({
  plugins: [react()],
  base: "./",
  build: { outDir: "../src/yektemak_fleet/web", emptyOutDir: true },
});
