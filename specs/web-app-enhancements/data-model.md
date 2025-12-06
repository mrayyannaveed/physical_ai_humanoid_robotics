# Data Model for Web Application Features

This document outlines the data models for the authentication, personalization, and language preference features of the web application.

## 1. User Model

Represents a user of the application.

-   **Entity Name**: `User`
-   **Fields**:
    -   `id`: UUID (Primary Key) - Unique identifier for the user.
    -   `username`: String (Unique, Indexed) - User's chosen username for login.
    -   `email`: String (Unique, Indexed) - User's email address.
    -   `hashed_password`: String - Hashed password for secure storage.
    -   `is_active`: Boolean (Default: `True`) - Account status.
    -   `created_at`: DateTime - Timestamp of user creation.
    -   `updated_at`: DateTime - Timestamp of last user update.
-   **Relationships**:
    -   One-to-One with `UserProfile`.
    -   One-to-Many with `UserPreference` (if preferences are stored separately per user and type).
-   **Validation Rules**:
    -   `username`: Minimum 3 characters, alphanumeric, no spaces.
    -   `email`: Valid email format, unique.
    -   `password`: Minimum 8 characters, includes uppercase, lowercase, number, and special character.

## 2. UserProfile Model

Stores personalization preferences for a user.

-   **Entity Name**: `UserProfile`
-   **Fields**:
    -   `id`: UUID (Primary Key) - Unique identifier for the profile.
    -   `user_id`: UUID (Foreign Key to `User.id`) - Links to the associated user.
    -   `preferred_language`: String (Default: `en`) - User's preferred language (e.g., 'en', 'ur').
    -   `theme`: String (Default: `dark`) - User's preferred theme (e.g., 'dark', 'light').
    -   `first_name`: String (Optional) - User's first name.
    -   `last_name`: String (Optional) - User's last name.
    -   `bio`: Text (Optional) - Short biography or description.
    -   `updated_at`: DateTime - Timestamp of last profile update.
-   **Relationships**:
    -   One-to-One with `User`.
-   **Validation Rules**:
    -   `preferred_language`: Must be a supported language code.
    -   `theme`: Must be a supported theme ('dark', 'light').

## 3. Translation Model (Optional, if dynamic translations are stored in DB)

If translations are managed dynamically through the database.

-   **Entity Name**: `Translation`
-   **Fields**:
    -   `id`: UUID (Primary Key)
    -   `key`: String (Unique, Indexed) - Key for the string (e.g., "welcome_message", "button_login").
    -   `language_code`: String (Indexed) - Language of the translation (e.g., 'en', 'ur').
    -   `value`: Text - Translated string content.
-   **Validation Rules**:
    -   `key`, `language_code` combination must be unique.

## Relationships Overview

-   A `User` HAS ONE `UserProfile`.
-   A `UserProfile` BELONGS TO ONE `User`.
-   `User` records are independent of `Translation` records.
-   `Translation` records are queried based on `key` and `language_code`.
