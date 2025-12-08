# Quickstart Guide: UI and Authentication Enhancements

This guide provides instructions for developers to quickly set up and verify the UI and authentication enhancements.

## Prerequisites

- Node.js (version 18 or higher)
- npm or yarn (preferred)
- Access to the existing backend API for authentication (no modifications to backend needed)
- Familiarity with Docusaurus and React development.

## Setup

1.  **Clone the repository (if you haven't already):**
    ```bash
    git clone [repository-url]
    cd physical_ai_native_book
    ```
2.  **Switch to the feature branch:**
    ```bash
    git checkout 1-ui-auth-enhancements
    ```
3.  **Install dependencies:**
    Navigate to the `book` directory and install the necessary frontend dependencies.
    ```bash
    cd book
    npm install # or yarn install
    cd ..
    ```
4.  **Start the Docusaurus development server:**
    From the `book` directory:
    ```bash
    npm start # or yarn start
    ```
    This will open the application in your browser, usually at `http://localhost:3000`.

## Verifying the Enhancements

### 1. Homepage UI Fixes

-   Open `http://localhost:3000` in your browser.
-   Resize the browser window to test responsiveness on different screen sizes (mobile, tablet, desktop).
-   Verify that the hero section (title, subtitle, and buttons) is perfectly centered, with balanced spacing and clean typography.
-   Ensure there are no stray elements or misalignments.

### 2. Button Functionality

-   On the homepage, click each of the following buttons and verify correct navigation:
    -   "Assessment"
    -   "GitHub"
    -   "Sign Up"
    -   "Login"
-   Hover over each button and confirm that the `cursor:pointer` style is applied and a visual hover effect is present.

### 3. Sign Up & Sign In

-   Click "Sign Up" on the homepage.
-   Fill out the registration form (email, password, etc.).
-   Proceed to the "Background Questions" step. Input some sample software/hardware background information.
-   Complete the registration. Verify that you are redirected to a logged-in state.
-   Click "Login" on the homepage.
-   Enter valid credentials for a registered user.
-   Verify successful login and redirection.
-   Test invalid inputs for both signup and signin forms (e.g., incorrect email format, wrong password) and confirm that appropriate client-side validation messages and error handling are displayed.
-   Observe loading states during API calls for both signup and signin.

### 4. Logged-in User Chapter Features

-   Log in to the application.
-   Navigate to any chapter page (e.g., `http://localhost:3000/docs/chapter-1/01-introduction`).
-   Verify that two buttons, "Personalize Content" and "Translate to Urdu", are visible at the start of the chapter content.
-   Click each button and observe that it triggers some frontend logic (e.g., a modal appears, a console log message, or content changes dynamically if existing logic is hooked up).
-   Log out and verify that these buttons are no longer visible on chapter pages for guest users.

## Troubleshooting

-   If the Docusaurus server doesn't start, check the console for errors. Ensure all dependencies are installed.
-   If authentication flows don't work, verify that the frontend is correctly communicating with the existing backend authentication endpoints. Check network requests in your browser's developer tools.
-   For UI issues, inspect element styles in developer tools to debug CSS.
