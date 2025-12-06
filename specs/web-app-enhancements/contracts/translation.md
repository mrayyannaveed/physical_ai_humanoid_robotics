# Translation API Contracts

## GET /translation/{language_code}

Retrieves translated text for a given language.

-   **Description**: Fetches a collection of translated strings for a specified language code. The keys can represent UI elements or content blocks.
-   **Path Parameters**:
    -   `language_code`: String (e.g., `en`, `ur`) - The two-letter ISO 639-1 code for the desired language.
-   **Responses**:
    -   `200 OK`: Translated strings successfully retrieved.
        ```json
        {
            "welcome_message": "Welcome!",
            "login_button": "Sign In",
            "homepage_title": "Home",
            "about_page_content": "This is the about page content in English."
            // ... more key-value pairs
        }
        ```
    -   `400 Bad Request`: Invalid `language_code` provided.
        ```json
        {
            "detail": "Invalid language code"
        }
        ```
    -   `404 Not Found`: No translations found for the specified language.
        ```json
        {
            "detail": "Translations for 'xx' not found"
        }
        ```
    -   `500 Internal Server Error`: Server error during translation retrieval.
        ```json
        {
            "detail": "Internal server error"
        }
        ```
