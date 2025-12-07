import React, { useState, useEffect } from 'react';
import { useI18n } from '../utils/i18n';

function Personalization() {
  const { t } = useI18n();
  const [personalizationSettings, setPersonalizationSettings] = useState({
    // Example settings:
    dashboardLayout: 'default',
    robotName: 'Humanoid',
    // ... other settings
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const [message, setMessage] = useState(null);

  // TODO: Fetch personalization settings from a backend API (e.g., /users/me/preferences)
  useEffect(() => {
    const fetchSettings = async () => {
      // Placeholder for API call
      // try {
      //   setIsLoading(true);
      //   const response = await fetch('/api/user/preferences', {
      //     headers: { 'Authorization': 'Bearer YOUR_TOKEN' }
      //   });
      //   const data = await response.json();
      //   if (response.ok) {
      //     setPersonalizationSettings(data);
      //   } else {
      //     setError(data.detail || 'Failed to fetch personalization settings.');
      //   }
      // } catch (err) {
      //   setError('Network error or server unavailable.');
      // } finally {
      //   setIsLoading(false);
      // }
    };
    fetchSettings();
  }, []);

  const handleSettingChange = (e) => {
    const { name, value } = e.target;
    setPersonalizationSettings(prevSettings => ({
      ...prevSettings,
      [name]: value,
    }));
  };

  const handleSaveSettings = async () => {
    // Placeholder for API call to save settings
    setMessage(null);
    setError(null);
    // try {
    //   setIsLoading(true);
    //   const response = await fetch('/api/user/preferences', {
    //     method: 'PUT',
    //     headers: {
    //       'Content-Type': 'application/json',
    //       'Authorization': 'Bearer YOUR_TOKEN',
    //     },
    //     body: JSON.stringify(personalizationSettings),
    //   });
    //   const data = await response.json();
    //   if (response.ok) {
    //     setMessage(t('preferences_updated'));
    //   } else {
    //     setError(data.detail || t('invalid_input'));
    //   }
    // } catch (err) {
    //   setError(t('network_error'));
    // } finally {
    //   setIsLoading(false);
    // }
  };


  return (
    <div className="card personalization-fade-in">
      <div className="card__header">
        <h3>{t('Personalization Settings')}</h3>
      </div>
      <div className="card__body">
        {isLoading && <p>Loading settings...</p>}
        {error && <div className="alert alert--danger">{error}</div>}
        {message && <div className="alert alert--success">{message}</div>}
        <div className="margin-bottom--sm">
          <label htmlFor="dashboardLayout" className="margin-right--sm">
            {t('Dashboard Layout')}:
          </label>
          <select
            id="dashboardLayout"
            name="dashboardLayout"
            value={personalizationSettings.dashboardLayout}
            onChange={handleSettingChange}
            className="select"
          >
            <option value="default">{t('Default')}</option>
            <option value="compact">{t('Compact')}</option>
            <option value="full">{t('Full')}</option>
          </select>
        </div>
        <div className="margin-bottom--sm">
          <label htmlFor="robotName" className="margin-right--sm">
            {t('Robot Name')}:
          </label>
          <input
            type="text"
            id="robotName"
            name="robotName"
            value={personalizationSettings.robotName}
            onChange={handleSettingChange}
            className="input"
          />
        </div>
        <button className="button button--primary" onClick={handleSaveSettings}>
          {t('Save Preferences')}
        </button>
      </div>
    </div>
  );
}

export default Personalization;
