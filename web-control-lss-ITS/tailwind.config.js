/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  darkMode: "class",
  theme: {
    extend: {
      fontFamily: {
        sans: ['"Plus Jakarta Sans"', 'sans-serif'],
        mono: ['"JetBrains Mono"', 'monospace'],
      },
      colors: {
        cockpit: {
          bg: '#f1f3f6',
          dark: '#14171d',
          card: '#ffffff',
          accent: '#0ea5e9',
          border: '#e2e8f0',
        }
      },
      boxShadow: {
        'cockpit': '0 10px 30px -5px rgba(0, 0, 0, 0.04), 0 4px 12px -2px rgba(0, 0, 0, 0.02)',
        'glow-active': '0 4px 14px 0 rgba(0, 0, 0, 0.15)',
        'glow-cyan': '0 0 20px -3px rgba(14, 165, 233, 0.45)',
      }
    },
  },
  plugins: [],
}
